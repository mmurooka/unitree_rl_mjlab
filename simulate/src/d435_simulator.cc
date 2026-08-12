#include "d435_simulator.h"

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "param.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

builtin_interfaces::msg::Time ToRosTime(double seconds)
{
  builtin_interfaces::msg::Time stamp;
  const auto ns = static_cast<std::int64_t>(std::llround(seconds * 1.0e9));
  stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
  return stamp;
}

sensor_msgs::msg::CameraInfo MakeCameraInfo(int width, int height, double hfov_deg,
                                            double vfov_deg, const std::string &frame)
{
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame;
  info.width = width;
  info.height = height;
  info.distortion_model = "plumb_bob";
  info.d.assign(5, 0.0);
  const double fx = width / (2.0 * std::tan(hfov_deg * kPi / 360.0));
  const double fy = height / (2.0 * std::tan(vfov_deg * kPi / 360.0));
  const double cx = (width - 1.0) / 2.0;
  const double cy = (height - 1.0) / 2.0;
  info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

geometry_msgs::msg::TransformStamped Transform(const std::string &parent, const std::string &child,
                                                double x, double y, double z,
                                                double qw, double qx, double qy, double qz)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.translation.z = z;
  tf.transform.rotation.w = qw;
  tf.transform.rotation.x = qx;
  tf.transform.rotation.y = qy;
  tf.transform.rotation.z = qz;
  return tf;
}
}  // namespace

class D435Simulator::Impl
{
public:
  struct Frame
  {
    double stamp = 0.0;
    double near_plane = 0.0;
    double far_plane = 0.0;
    int depth_map = mjDEPTH_ZERONEAR;
    std::vector<std::uint8_t> rgb;
    std::vector<float> raw_depth;
    std::vector<float> aligned_depth;
  };

  explicit Impl(const mjModel *model)
      : width_(param::config.d435_width), height_(param::config.d435_height),
        period_(1.0 / param::config.d435_frequency),
        color_camera_id_(mj_name2id(model, mjOBJ_CAMERA, "camera_color_optical_frame")),
        depth_camera_id_(mj_name2id(model, mjOBJ_CAMERA, "camera_depth_optical_frame")),
        color_info_(MakeCameraInfo(width_, height_, 69.0, 54.48, "camera_color_optical_frame")),
        depth_info_(MakeCameraInfo(width_, height_, 75.0, 60.39, "camera_depth_optical_frame"))
  {
    if (width_ <= 0 || height_ <= 0 || param::config.d435_frequency <= 0.0 ||
        param::config.d435_pointcloud_downsample < 1 || color_camera_id_ < 0 ||
        depth_camera_id_ < 0)
    {
      throw std::runtime_error("invalid D435 configuration or camera missing from MJCF");
    }
    mjv_defaultScene(&scene_);
    mjv_makeScene(model, &scene_, 100000);
    mjv_defaultOption(&option_);
    mjv_defaultCamera(&camera_);
    camera_.type = mjCAMERA_FIXED;

    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(argc, nullptr);
      owns_ros_context_ = true;
    }
    node_ = std::make_shared<rclcpp::Node>("unitree_mujoco_d435");
    const auto qos = rclcpp::SensorDataQoS();
    const auto &root = param::config.d435_topic_root;
    color_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(root + "/color/image_raw", qos);
    color_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(root + "/color/camera_info", qos);
    depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(root + "/depth/image_rect_raw", qos);
    depth_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(root + "/depth/camera_info", qos);
    aligned_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(root + "/aligned_depth_to_color/image_raw", qos);
    aligned_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(root + "/aligned_depth_to_color/camera_info", qos);
    if (param::config.d435_publish_pointcloud)
    {
      cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(root + "/depth/color/points", qos);
    }
    tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
    PublishStaticTransforms();

    rgb_.resize(static_cast<std::size_t>(width_) * height_ * 3);
    raw_depth_mm_.resize(static_cast<std::size_t>(width_) * height_);
    aligned_depth_mm_.resize(static_cast<std::size_t>(width_) * height_);
    worker_thread_ = std::thread(&Impl::WorkerLoop, this);
    std::cout << "D435: publishing " << width_ << "x" << height_ << " RGB-D at "
              << param::config.d435_frequency << " Hz under " << root << std::endl;
  }

  ~Impl()
  {
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      stop_ = true;
    }
    frame_cv_.notify_all();
    if (worker_thread_.joinable())
    {
      worker_thread_.join();
    }
    mjv_freeScene(&scene_);
    tf_broadcaster_.reset();
    node_.reset();
    if (owns_ros_context_ && rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void Render(const mjModel *model, mjData *data, mjrContext *context)
  {
    if (data->time < last_stamp_)
    {
      last_stamp_ = -1.0;
    }
    if (last_stamp_ >= 0.0 && data->time - last_stamp_ < period_)
    {
      return;
    }
    last_stamp_ = data->time;

    std::unique_ptr<Frame> frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      frame = free_frame_ ? std::move(free_frame_) : std::make_unique<Frame>();
    }
    frame->stamp = data->time;
    frame->near_plane = model->vis.map.znear * model->stat.extent;
    frame->far_plane = model->vis.map.zfar * model->stat.extent;
    frame->depth_map = context->readDepthMap;
    frame->rgb.resize(static_cast<std::size_t>(width_) * height_ * 3);
    frame->raw_depth.resize(static_cast<std::size_t>(width_) * height_);
    frame->aligned_depth.resize(static_cast<std::size_t>(width_) * height_);

    const mjrRect viewport{0, 0, width_, height_};
    mjr_setBuffer(mjFB_OFFSCREEN, context);

    camera_.fixedcamid = color_camera_id_;
    mjv_updateScene(model, data, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
    mjr_render(viewport, &scene_, context);
    mjr_readPixels(frame->rgb.data(), frame->aligned_depth.data(), viewport, context);

    camera_.fixedcamid = depth_camera_id_;
    mjv_updateScene(model, data, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
    mjr_render(viewport, &scene_, context);
    mjr_readPixels(nullptr, frame->raw_depth.data(), viewport, context);

    mjr_setBuffer(mjFB_WINDOW, context);
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (pending_frame_)
      {
        free_frame_ = std::move(pending_frame_);
      }
      pending_frame_ = std::move(frame);
    }
    frame_cv_.notify_one();
  }

private:
  void ConvertDepth(const Frame &frame, const std::vector<float> &input,
                    std::vector<std::uint16_t> &output)
  {
    for (int y = 0; y < height_; ++y)
    {
      for (int x = 0; x < width_; ++x)
      {
        const float sample = input[static_cast<std::size_t>(height_ - 1 - y) * width_ + x];
        const double normalized = frame.depth_map == mjDEPTH_ZEROFAR ? 1.0 - sample : sample;
        const double meters = frame.near_plane * frame.far_plane /
                              (frame.far_plane - normalized * (frame.far_plane - frame.near_plane));
        const auto index = static_cast<std::size_t>(y) * width_ + x;
        output[index] = meters >= param::config.d435_min_range && meters <= param::config.d435_max_range
                            ? static_cast<std::uint16_t>(std::min(65535.0, std::round(meters * 1000.0)))
                            : 0;
      }
    }
  }

  void FlipRgb(const std::vector<std::uint8_t> &input)
  {
    const std::size_t row_size = static_cast<std::size_t>(width_) * 3;
    for (int y = 0; y < height_; ++y)
    {
      const auto *source = input.data() + static_cast<std::size_t>(height_ - 1 - y) * row_size;
      auto *destination = rgb_.data() + static_cast<std::size_t>(y) * row_size;
      std::memcpy(destination, source, row_size);
    }
  }

  void WorkerLoop()
  {
    while (true)
    {
      std::unique_ptr<Frame> frame;
      {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_cv_.wait(lock, [this] { return stop_ || pending_frame_; });
        if (stop_)
        {
          return;
        }
        frame = std::move(pending_frame_);
      }

      FlipRgb(frame->rgb);
      ConvertDepth(*frame, frame->raw_depth, raw_depth_mm_);
      ConvertDepth(*frame, frame->aligned_depth, aligned_depth_mm_);
      Publish(frame->stamp);

      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!free_frame_)
      {
        free_frame_ = std::move(frame);
      }
    }
  }

  sensor_msgs::msg::Image MakeImage(const builtin_interfaces::msg::Time &stamp, const std::string &frame,
                                    const std::string &encoding, std::uint32_t step,
                                    const std::uint8_t *data, std::size_t size) const
  {
    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = frame;
    image.height = height_;
    image.width = width_;
    image.encoding = encoding;
    image.is_bigendian = false;
    image.step = step;
    image.data.assign(data, data + size);
    return image;
  }

  sensor_msgs::msg::PointCloud2 MakeCloud(const builtin_interfaces::msg::Time &stamp) const
  {
    using sensor_msgs::msg::PointField;
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = "camera_color_optical_frame";
    const int stride = param::config.d435_pointcloud_downsample;
    const int sampled_width = (width_ + stride - 1) / stride;
    const int sampled_height = (height_ + stride - 1) / stride;
    cloud.height = 1;
    cloud.width = sampled_width * sampled_height;
    cloud.fields = {
        PointField().set__name("x").set__offset(0).set__datatype(PointField::FLOAT32).set__count(1),
        PointField().set__name("y").set__offset(4).set__datatype(PointField::FLOAT32).set__count(1),
        PointField().set__name("z").set__offset(8).set__datatype(PointField::FLOAT32).set__count(1),
        PointField().set__name("rgb").set__offset(12).set__datatype(PointField::FLOAT32).set__count(1)};
    cloud.point_step = 16;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    cloud.data.resize(cloud.row_step);
    const double fx = color_info_.k[0], fy = color_info_.k[4];
    const double cx = color_info_.k[2], cy = color_info_.k[5];
    std::size_t point = 0;
    for (int y = 0; y < height_; y += stride)
    {
      for (int x = 0; x < width_; x += stride, ++point)
      {
        const auto pixel = static_cast<std::size_t>(y) * width_ + x;
        const float z = aligned_depth_mm_[pixel] * 0.001F;
        const float px = z > 0.0F ? static_cast<float>((x - cx) * z / fx) : std::numeric_limits<float>::quiet_NaN();
        const float py = z > 0.0F ? static_cast<float>((y - cy) * z / fy) : std::numeric_limits<float>::quiet_NaN();
        const std::uint32_t rgb = (static_cast<std::uint32_t>(rgb_[3 * pixel]) << 16) |
                                  (static_cast<std::uint32_t>(rgb_[3 * pixel + 1]) << 8) |
                                  rgb_[3 * pixel + 2];
        auto *dst = cloud.data.data() + point * cloud.point_step;
        std::memcpy(dst, &px, 4); std::memcpy(dst + 4, &py, 4); std::memcpy(dst + 8, &z, 4);
        std::memcpy(dst + 12, &rgb, 4);
      }
    }
    return cloud;
  }

  void Publish(double time)
  {
    const auto stamp = ToRosTime(time);
    auto color = MakeImage(stamp, "camera_color_optical_frame", "rgb8", width_ * 3,
                           rgb_.data(), rgb_.size());
    auto depth = MakeImage(stamp, "camera_depth_optical_frame", "16UC1", width_ * 2,
                           reinterpret_cast<const std::uint8_t *>(raw_depth_mm_.data()), raw_depth_mm_.size() * 2);
    auto aligned = MakeImage(stamp, "camera_color_optical_frame", "16UC1", width_ * 2,
                             reinterpret_cast<const std::uint8_t *>(aligned_depth_mm_.data()), aligned_depth_mm_.size() * 2);
    color_info_.header.stamp = depth_info_.header.stamp = stamp;
    auto aligned_info = color_info_;
    color_pub_->publish(color); color_info_pub_->publish(color_info_);
    depth_pub_->publish(depth); depth_info_pub_->publish(depth_info_);
    aligned_pub_->publish(aligned); aligned_info_pub_->publish(aligned_info);
    if (cloud_pub_) cloud_pub_->publish(MakeCloud(stamp));
  }

  void PublishStaticTransforms()
  {
    const auto stamp = node_->now();
    std::vector<geometry_msgs::msg::TransformStamped> transforms = {
        Transform("camera_link", "camera_depth_frame", 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        Transform("camera_depth_frame", "camera_depth_optical_frame", 0.0, 0.0, 0.0, 0.5, -0.5, 0.5, -0.5),
        Transform("camera_link", "camera_color_frame", 0.015, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        Transform("camera_color_frame", "camera_color_optical_frame", 0.0, 0.0, 0.0, 0.5, -0.5, 0.5, -0.5)};
    for (auto &tf : transforms) tf.header.stamp = stamp;
    tf_broadcaster_->sendTransform(transforms);
  }

  int width_, height_;
  double period_, last_stamp_ = -1.0;
  int color_camera_id_, depth_camera_id_;
  mjvScene scene_{};
  mjvOption option_{};
  mjvCamera camera_{};
  std::vector<std::uint8_t> rgb_;
  std::vector<std::uint16_t> raw_depth_mm_, aligned_depth_mm_;
  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  std::unique_ptr<Frame> pending_frame_, free_frame_;
  bool stop_ = false;
  std::thread worker_thread_;
  sensor_msgs::msg::CameraInfo color_info_, depth_info_;
  bool owns_ros_context_ = false;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_, depth_pub_, aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_, depth_info_pub_, aligned_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

D435Simulator::D435Simulator(const mjModel *model) : impl_(std::make_unique<Impl>(model)) {}
D435Simulator::~D435Simulator() = default;
bool D435Simulator::valid() const { return impl_ != nullptr; }
void D435Simulator::render(const mjModel *model, mjData *data, mjrContext *context)
{
  impl_->Render(model, data, context);
}

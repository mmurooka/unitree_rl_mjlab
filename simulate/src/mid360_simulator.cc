#include "mid360_simulator.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "param.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMaxQueuedClouds = 2;

struct RayDirection
{
  float x;
  float y;
  float z;
  std::uint8_t line;
};

struct LivoxPoint
{
  float x;
  float y;
  float z;
  float intensity;
  std::uint8_t tag;
  std::uint8_t line;
  double timestamp;
};

struct Cloud
{
  double stamp = 0.0;
  std::vector<LivoxPoint> points;
};

builtin_interfaces::msg::Time ToRosTime(double seconds)
{
  builtin_interfaces::msg::Time stamp;
  const auto nanoseconds = static_cast<std::int64_t>(std::llround(seconds * 1.0e9));
  stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

std::vector<RayDirection> LoadScanPattern(const char *path)
{
  std::ifstream input(path);
  if (!input)
  {
    throw std::runtime_error(std::string("cannot open MID-360 scan pattern: ") + path);
  }

  std::vector<RayDirection> rays;
  std::string line;
  std::getline(input, line);  // Time/s,Azimuth/deg,Zenith/deg
  while (std::getline(input, line))
  {
    std::istringstream row(line);
    std::string time_text;
    std::string azimuth_text;
    std::string zenith_text;
    if (!std::getline(row, time_text, ',') || !std::getline(row, azimuth_text, ',') ||
        !std::getline(row, zenith_text, ','))
    {
      continue;
    }

    const double azimuth = std::stod(azimuth_text) * kPi / 180.0;
    const double elevation = (90.0 - std::stod(zenith_text)) * kPi / 180.0;
    const double cos_elevation = std::cos(elevation);
    rays.push_back({
        static_cast<float>(cos_elevation * std::cos(azimuth)),
        static_cast<float>(cos_elevation * std::sin(azimuth)),
        static_cast<float>(std::sin(elevation)),
        static_cast<std::uint8_t>(rays.size() % 4),
    });
  }
  if (rays.empty())
  {
    throw std::runtime_error("MID-360 scan pattern contains no rays");
  }
  return rays;
}
}  // namespace

class Mid360Simulator::Impl
{
public:
  Impl(const mjModel *model, const char *scan_pattern_path)
      : rays_(LoadScanPattern(scan_pattern_path)),
        lidar_site_id_(mj_name2id(model, mjOBJ_SITE, param::config.mid360_frame_id.c_str())),
        lidar_body_id_(lidar_site_id_ >= 0 ? model->site_bodyid[lidar_site_id_] : -1),
        gyro_sensor_id_(mj_name2id(model, mjOBJ_SENSOR, "livox_gyro")),
        acc_sensor_id_(mj_name2id(model, mjOBJ_SENSOR, "livox_acc"))
  {
    if (lidar_site_id_ < 0)
    {
      throw std::runtime_error("MID-360 site not found in model: " + param::config.mid360_frame_id);
    }
    if (param::config.mid360_point_rate <= 0.0 || param::config.mid360_publish_frequency <= 0.0 ||
        param::config.mid360_downsample < 1)
    {
      throw std::runtime_error("invalid MID-360 rate or downsample configuration");
    }

    points_per_second_ = param::config.mid360_point_rate / param::config.mid360_downsample;
    cloud_period_ = 1.0 / param::config.mid360_publish_frequency;
    imu_period_ = 1.0 / param::config.mid360_imu_frequency;
    active_cloud_.points.reserve(static_cast<std::size_t>(points_per_second_ * cloud_period_ * 1.1));

    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(argc, nullptr);
      owns_ros_context_ = true;
    }
    node_ = std::make_shared<rclcpp::Node>("unitree_mujoco_mid360");
    cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        param::config.mid360_lidar_topic, rclcpp::SensorDataQoS());
    imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>(
        param::config.mid360_imu_topic, rclcpp::SensorDataQoS());
    publisher_thread_ = std::thread(&Impl::PublishLoop, this);

    std::cout << "MID-360: loaded " << rays_.size() << " official scan directions, publishing "
              << param::config.mid360_lidar_topic << " at "
              << param::config.mid360_publish_frequency << " Hz" << std::endl;
  }

  ~Impl()
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stop_ = true;
    }
    queue_cv_.notify_all();
    if (publisher_thread_.joinable())
    {
      publisher_thread_.join();
    }
    node_.reset();
    if (owns_ros_context_ && rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void Advance(const mjModel *model, mjData *data)
  {
    if (data->time < last_sim_time_)
    {
      ray_budget_ = 0.0;
      active_cloud_.points.clear();
      active_cloud_.stamp = data->time;
      next_cloud_time_ = data->time + cloud_period_;
      next_imu_time_ = data->time;
    }

    const double dt = std::max(0.0, data->time - last_sim_time_);
    last_sim_time_ = data->time;
    if (next_cloud_time_ < 0.0)
    {
      active_cloud_.stamp = data->time;
      next_cloud_time_ = data->time + cloud_period_;
      next_imu_time_ = data->time;
    }

    ray_budget_ += dt * points_per_second_;
    const int ray_count = static_cast<int>(ray_budget_);
    ray_budget_ -= ray_count;
    CastRays(model, data, ray_count);

    while (data->time >= next_imu_time_)
    {
      QueueImu(model, data, next_imu_time_);
      next_imu_time_ += imu_period_;
    }
    if (data->time >= next_cloud_time_)
    {
      const double next_cloud_start = next_cloud_time_;
      QueueCloud(std::move(active_cloud_));
      active_cloud_ = Cloud{};
      active_cloud_.points.reserve(static_cast<std::size_t>(points_per_second_ * cloud_period_ * 1.1));
      do
      {
        next_cloud_time_ += cloud_period_;
      } while (data->time >= next_cloud_time_);
      active_cloud_.stamp = next_cloud_start;
    }
  }

private:
  void CastRays(const mjModel *model, mjData *data, int ray_count)
  {
    if (ray_count <= 0)
    {
      return;
    }

    std::vector<mjtNum> world_directions(static_cast<std::size_t>(ray_count) * 3);
    std::vector<RayDirection> local_directions(static_cast<std::size_t>(ray_count));
    const mjtNum *rotation = data->site_xmat + 9 * lidar_site_id_;
    for (int i = 0; i < ray_count; ++i)
    {
      const RayDirection local = rays_[ray_index_];
      ray_index_ = (ray_index_ + param::config.mid360_downsample) % rays_.size();
      local_directions[i] = local;
      for (int axis = 0; axis < 3; ++axis)
      {
        world_directions[3 * i + axis] = rotation[3 * axis] * local.x +
                                         rotation[3 * axis + 1] * local.y +
                                         rotation[3 * axis + 2] * local.z;
      }
    }

    std::vector<int> geom_ids(static_cast<std::size_t>(ray_count), -1);
    std::vector<mjtNum> distances(static_cast<std::size_t>(ray_count), -1.0);
    const mjtNum *origin = data->site_xpos + 3 * lidar_site_id_;
    mj_multiRay(model, data, origin, world_directions.data(), nullptr, 1, lidar_body_id_,
                geom_ids.data(), distances.data(), ray_count, param::config.mid360_max_range);

    const double first_stamp = data->time - (ray_count - 1) / points_per_second_;
    for (int i = 0; i < ray_count; ++i)
    {
      const double distance = distances[i];
      if (geom_ids[i] < 0 || distance < param::config.mid360_min_range ||
          distance > param::config.mid360_max_range)
      {
        continue;
      }
      const auto &ray = local_directions[i];
      active_cloud_.points.push_back({
          static_cast<float>(ray.x * distance), static_cast<float>(ray.y * distance),
          static_cast<float>(ray.z * distance), 100.0F, 0, ray.line,
          first_stamp + i / points_per_second_,
      });
    }
  }

  void QueueCloud(Cloud cloud)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (cloud_queue_.size() == kMaxQueuedClouds)
    {
      cloud_queue_.pop_front();
    }
    cloud_queue_.push_back(std::move(cloud));
    queue_cv_.notify_one();
  }

  void QueueImu(const mjModel *model, const mjData *data, double stamp)
  {
    if (gyro_sensor_id_ < 0 || acc_sensor_id_ < 0)
    {
      return;
    }
    sensor_msgs::msg::Imu message;
    message.header.stamp = ToRosTime(stamp);
    message.header.frame_id = param::config.mid360_frame_id;
    message.orientation_covariance[0] = -1.0;
    const mjtNum *gyro = data->sensordata + model->sensor_adr[gyro_sensor_id_];
    const mjtNum *acc = data->sensordata + model->sensor_adr[acc_sensor_id_];
    message.angular_velocity.x = gyro[0];
    message.angular_velocity.y = gyro[1];
    message.angular_velocity.z = gyro[2];
    message.linear_acceleration.x = acc[0];
    message.linear_acceleration.y = acc[1];
    message.linear_acceleration.z = acc[2];
    std::lock_guard<std::mutex> lock(queue_mutex_);
    latest_imu_ = std::move(message);
    has_imu_ = true;
    queue_cv_.notify_one();
  }

  sensor_msgs::msg::PointCloud2 MakeCloudMessage(const Cloud &cloud) const
  {
    using sensor_msgs::msg::PointField;
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = ToRosTime(cloud.stamp);
    message.header.frame_id = param::config.mid360_frame_id;
    message.height = 1;
    message.width = static_cast<std::uint32_t>(cloud.points.size());
    message.fields = {
        PointField().set__name("x").set__offset(0).set__datatype(PointField::FLOAT32).set__count(1),
        PointField().set__name("y").set__offset(4).set__datatype(PointField::FLOAT32).set__count(1),
        PointField().set__name("z").set__offset(8).set__datatype(PointField::FLOAT32).set__count(1),
        PointField().set__name("intensity").set__offset(12).set__datatype(PointField::FLOAT32).set__count(1),
        PointField().set__name("tag").set__offset(16).set__datatype(PointField::UINT8).set__count(1),
        PointField().set__name("line").set__offset(17).set__datatype(PointField::UINT8).set__count(1),
        PointField().set__name("timestamp").set__offset(18).set__datatype(PointField::FLOAT64).set__count(1),
    };
    message.is_bigendian = false;
    message.point_step = 26;
    message.row_step = message.point_step * message.width;
    message.data.resize(message.row_step);
    message.is_dense = true;
    for (std::size_t i = 0; i < cloud.points.size(); ++i)
    {
      const auto &point = cloud.points[i];
      auto *dst = message.data.data() + i * message.point_step;
      std::memcpy(dst, &point.x, sizeof(float));
      std::memcpy(dst + 4, &point.y, sizeof(float));
      std::memcpy(dst + 8, &point.z, sizeof(float));
      std::memcpy(dst + 12, &point.intensity, sizeof(float));
      dst[16] = point.tag;
      dst[17] = point.line;
      const double relative_timestamp = point.timestamp - cloud.stamp;
      std::memcpy(dst + 18, &relative_timestamp, sizeof(double));
    }
    return message;
  }

  void PublishLoop()
  {
    while (true)
    {
      Cloud cloud;
      sensor_msgs::msg::Imu imu;
      bool has_cloud = false;
      bool has_imu = false;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait_for(lock, std::chrono::milliseconds(10), [this] {
          return stop_ || !cloud_queue_.empty() || has_imu_;
        });
        if (stop_)
        {
          return;
        }
        if (!cloud_queue_.empty())
        {
          cloud = std::move(cloud_queue_.front());
          cloud_queue_.pop_front();
          has_cloud = true;
        }
        if (has_imu_)
        {
          imu = latest_imu_;
          has_imu_ = false;
          has_imu = true;
        }
      }
      if (has_imu)
      {
        imu_publisher_->publish(imu);
      }
      if (has_cloud)
      {
        cloud_publisher_->publish(MakeCloudMessage(cloud));
      }
    }
  }

  std::vector<RayDirection> rays_;
  std::size_t ray_index_ = 0;
  int lidar_site_id_ = -1;
  int lidar_body_id_ = -1;
  int gyro_sensor_id_ = -1;
  int acc_sensor_id_ = -1;
  double points_per_second_ = 0.0;
  double cloud_period_ = 0.1;
  double imu_period_ = 0.005;
  double last_sim_time_ = 0.0;
  double next_cloud_time_ = -1.0;
  double next_imu_time_ = -1.0;
  double ray_budget_ = 0.0;
  Cloud active_cloud_;

  bool owns_ros_context_ = false;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  std::thread publisher_thread_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<Cloud> cloud_queue_;
  sensor_msgs::msg::Imu latest_imu_;
  bool has_imu_ = false;
  bool stop_ = false;
};

Mid360Simulator::Mid360Simulator(const mjModel *model, const char *scan_pattern_path)
{
  try
  {
    impl_ = std::make_unique<Impl>(model, scan_pattern_path);
  }
  catch (const std::exception &error)
  {
    std::cerr << "MID-360 disabled: " << error.what() << std::endl;
  }
}

Mid360Simulator::~Mid360Simulator() = default;

bool Mid360Simulator::valid() const
{
  return impl_ != nullptr;
}

void Mid360Simulator::advance(const mjModel *model, mjData *data)
{
  if (impl_)
  {
    impl_->Advance(model, data);
  }
}

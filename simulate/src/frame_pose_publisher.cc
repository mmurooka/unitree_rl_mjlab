#include "frame_pose_publisher.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "param.h"

namespace
{
std::string JsonEscape(const std::string &value)
{
  std::ostringstream output;
  for (const unsigned char character : value)
  {
    switch (character)
    {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20)
        {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(character) << std::dec;
        }
        else
        {
          output << character;
        }
    }
  }
  return output.str();
}

bool IsRobotBody(const mjModel *model, int body_id, int robot_root_body_id)
{
  while (body_id > 0)
  {
    if (body_id == robot_root_body_id)
    {
      return true;
    }
    body_id = model->body_parentid[body_id];
  }
  return false;
}
}  // namespace

class FramePosePublisher::Impl
{
public:
  explicit Impl(const mjModel *model)
      : period_(1.0 / param::config.frame_pose_publish_frequency),
        reference_site_id_(mj_name2id(
            model, mjOBJ_SITE, param::config.frame_pose_reference_site.c_str())),
        camera_body_id_(mj_name2id(model, mjOBJ_BODY, "camera_link")),
        publish_on_primary_topic_(
            param::config.frame_pose_source == "ground_truth"),
        output_topic_(
            publish_on_primary_topic_
                ? param::config.frame_pose_topic
                : param::config.frame_pose_ground_truth_topic)
  {
    if (output_topic_.empty() ||
        param::config.frame_pose_publish_frequency <= 0.0)
    {
      throw std::runtime_error("invalid frame-pose-list topic or publish frequency");
    }
    if (reference_site_id_ < 0)
    {
      throw std::runtime_error(
          "frame-pose-list reference site not found in MuJoCo model: " +
          param::config.frame_pose_reference_site);
    }
    if (param::config.frame_pose_source != "ground_truth" &&
        param::config.frame_pose_source != "aruco")
    {
      throw std::runtime_error(
          "frame-pose-list source must be 'ground_truth' or 'aruco': " +
          param::config.frame_pose_source);
    }
    if (param::config.frame_pose_reference_frame.empty())
    {
      throw std::runtime_error("frame-pose-list reference frame must not be empty");
    }
    if (camera_body_id_ < 0 && param::config.frame_pose_source == "aruco")
    {
      throw std::runtime_error("camera_link body not found in MuJoCo model");
    }

    const int robot_root_body_id = model->site_bodyid[reference_site_id_];
    for (int body_id = 1; body_id < model->nbody; ++body_id)
    {
      const char *name = mj_id2name(model, mjOBJ_BODY, body_id);
      if (name != nullptr && name[0] != '\0' &&
          !IsRobotBody(model, body_id, robot_root_body_id))
      {
        frames_.push_back({body_id, name});
      }
    }

    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(
          argc, nullptr, rclcpp::InitOptions(),
          rclcpp::SignalHandlerOptions::None);
      owns_ros_context_ = true;
    }
    node_ = std::make_shared<rclcpp::Node>("unitree_mujoco_frame_pose_list");
    camera_tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    publisher_ = node_->create_publisher<std_msgs::msg::String>(
        output_topic_, rclcpp::QoS(1));

    std::cout << "Frame pose list source: " << param::config.frame_pose_source;
    if (publish_on_primary_topic_)
    {
      std::cout << "; publishing " << frames_.size()
                << " non-robot MuJoCo bodies on "
                << output_topic_;
    }
    else
    {
      std::cout << "; publishing simulation truth for marker search on "
                << output_topic_;
    }
    if (camera_body_id_ >= 0)
    {
      std::cout << "; broadcasting "
                << param::config.frame_pose_reference_frame
                << " -> camera_link";
    }
    std::cout << " at " << param::config.frame_pose_publish_frequency << " Hz"
              << std::endl;
  }

  ~Impl()
  {
    publisher_.reset();
    camera_tf_broadcaster_.reset();
    node_.reset();
    if (owns_ros_context_ && rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void Advance(const mjModel *model, const mjData *data)
  {
    if (data->time < last_sim_time_)
    {
      next_publish_time_ = data->time;
    }
    last_sim_time_ = data->time;
    if (data->time + 1.0e-12 < next_publish_time_)
    {
      return;
    }
    do
    {
      next_publish_time_ += period_;
    } while (next_publish_time_ <= data->time);

    const mjtNum *reference_position =
        data->site_xpos + 3 * reference_site_id_;
    const mjtNum *reference_rotation =
        data->site_xmat + 9 * reference_site_id_;
    mjtNum reference_quaternion[4];
    mju_mat2Quat(reference_quaternion, reference_rotation);
    const double reference_yaw = std::atan2(
        2.0 * (reference_quaternion[0] * reference_quaternion[3] +
               reference_quaternion[1] * reference_quaternion[2]),
        1.0 - 2.0 * (reference_quaternion[2] * reference_quaternion[2] +
                     reference_quaternion[3] * reference_quaternion[3]));
    const mjtNum reference_yaw_quaternion[4] = {
        std::cos(0.5 * reference_yaw),
        0.0,
        0.0,
        std::sin(0.5 * reference_yaw),
    };
    const mjtNum inverse_reference_quaternion[4] = {
        reference_yaw_quaternion[0],
        -reference_yaw_quaternion[1],
        -reference_yaw_quaternion[2],
        -reference_yaw_quaternion[3],
    };
    const mjtNum cos_yaw = std::cos(reference_yaw);
    const mjtNum sin_yaw = std::sin(reference_yaw);
    const mjtNum reference_yaw_rotation[9] = {
        cos_yaw, -sin_yaw, 0.0,
        sin_yaw, cos_yaw, 0.0,
        0.0, 0.0, 1.0,
    };

    if (camera_body_id_ >= 0)
    {
      const mjtNum *camera_world_position = data->xpos + 3 * camera_body_id_;
      const mjtNum camera_world_delta[3] = {
          camera_world_position[0] - reference_position[0],
          camera_world_position[1] - reference_position[1],
          camera_world_position[2] - reference_position[2],
      };
      mjtNum camera_relative_position[3];
      mju_mulMatTVec3(
          camera_relative_position, reference_yaw_rotation,
          camera_world_delta);
      mjtNum camera_relative_quaternion[4];
      mju_mulQuat(
          camera_relative_quaternion, inverse_reference_quaternion,
          data->xquat + 4 * camera_body_id_);

      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = node_->now();
      transform.header.frame_id = param::config.frame_pose_reference_frame;
      transform.child_frame_id = "camera_link";
      transform.transform.translation.x = camera_relative_position[0];
      transform.transform.translation.y = camera_relative_position[1];
      transform.transform.translation.z = camera_relative_position[2];
      transform.transform.rotation.w = camera_relative_quaternion[0];
      transform.transform.rotation.x = camera_relative_quaternion[1];
      transform.transform.rotation.y = camera_relative_quaternion[2];
      transform.transform.rotation.z = camera_relative_quaternion[3];
      camera_tf_broadcaster_->sendTransform(transform);
    }

    std::ostringstream json;
    json << std::setprecision(17)
         << "{\"stamp\":" << data->time
         << ",\"reference_frame\":\""
         << JsonEscape(param::config.frame_pose_reference_frame)
         << "\",\"frame_pose_list\":[";
    bool first = true;
    for (const auto &[body_id, frame_name] : frames_)
    {
      const mjtNum *world_position = data->xpos + 3 * body_id;
      const mjtNum world_delta[3] = {
          world_position[0] - reference_position[0],
          world_position[1] - reference_position[1],
          world_position[2] - reference_position[2],
      };
      mjtNum relative_position[3];
      // Navigation goals use the gravity-aligned robot frame captured on
      // entry, so remove roll/pitch rather than projecting through them.
      mju_mulMatTVec3(
          relative_position, reference_yaw_rotation, world_delta);

      mjtNum relative_quaternion[4];
      mju_mulQuat(
          relative_quaternion,
          inverse_reference_quaternion,
          data->xquat + 4 * body_id);
      if (!std::isfinite(relative_position[0]) ||
          !std::isfinite(relative_position[1]) ||
          !std::isfinite(relative_position[2]) ||
          !std::isfinite(relative_quaternion[0]) ||
          !std::isfinite(relative_quaternion[1]) ||
          !std::isfinite(relative_quaternion[2]) ||
          !std::isfinite(relative_quaternion[3]))
      {
        continue;
      }

      if (!first)
      {
        json << ',';
      }
      first = false;
      json << "{\"frame_name\":\"" << JsonEscape(frame_name)
           << "\",\"rel_pose\":["
           << relative_position[0] << ','
           << relative_position[1] << ','
           << relative_position[2] << ','
           << relative_quaternion[1] << ','
           << relative_quaternion[2] << ','
           << relative_quaternion[3] << ','
           << relative_quaternion[0] << "]}";
    }
    json << "]}";

    std_msgs::msg::String message;
    message.data = json.str();
    publisher_->publish(std::move(message));
  }

private:
  double period_;
  int reference_site_id_;
  int camera_body_id_;
  bool publish_on_primary_topic_;
  std::string output_topic_;
  double last_sim_time_ = -1.0;
  double next_publish_time_ = 0.0;
  bool owns_ros_context_ = false;
  std::vector<std::pair<int, std::string>> frames_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> camera_tf_broadcaster_;
};

FramePosePublisher::FramePosePublisher(const mjModel *model)
    : impl_(std::make_unique<Impl>(model))
{
}

FramePosePublisher::~FramePosePublisher() = default;

void FramePosePublisher::advance(const mjModel *model, const mjData *data)
{
  impl_->Advance(model, data);
}

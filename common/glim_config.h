#pragma once

#include <eigen3/Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

// Read the SAME external directory passed to glim_rosnode's config_path.
// Never rewrites/merges settings or starts GLIM. JSON comments are supported.
namespace g1 {
struct GlimConfig {
    Eigen::Isometry3d lidar_to_imu = Eigen::Isometry3d::Identity();
    std::string imu_frame, lidar_frame, odom_frame;
    std::string imu_topic, lidar_topic;
    double acc_scale;
};
inline nlohmann::json read_glim_json(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Cannot read external GLIM config: " + path.string());
    return nlohmann::json::parse(stream, nullptr, true, true);
}
inline GlimConfig load_glim_config(const std::filesystem::path& dir) {
    const auto global = read_glim_json(dir / "config.json").at("global");
    const auto sensors = read_glim_json(dir / global.at("config_sensors").get<std::string>()).at("sensors");
    const auto ros = read_glim_json(dir / global.at("config_ros").get<std::string>()).at("glim_ros");
    const auto values = sensors.at("T_lidar_imu").get<std::vector<double>>();
    if (values.size() != 7) throw std::runtime_error("T_lidar_imu must be xyz, qx,qy,qz,qw");
    for (double v : values) if (!std::isfinite(v)) throw std::runtime_error("Nonfinite T_lidar_imu");
    Eigen::Quaterniond q(values[6], values[3], values[4], values[5]);
    if (q.norm() < 1e-9) throw std::runtime_error("Zero T_lidar_imu quaternion");
    GlimConfig config;
    config.lidar_to_imu.linear() = q.normalized().toRotationMatrix();
    config.lidar_to_imu.translation() = Eigen::Vector3d(values[0],values[1],values[2]);
    config.imu_frame = ros.value("imu_frame_id", "");
    config.lidar_frame = ros.value("lidar_frame_id", "");
    // Standard Livox driver output frame names when GLIM auto-detects labels.
    if (config.imu_frame.empty()) config.imu_frame = "livox_frame";
    if (config.lidar_frame.empty()) config.lidar_frame = "livox_frame";
    config.odom_frame = ros.value("odom_frame_id", "odom");
    config.imu_topic = ros.at("imu_topic").get<std::string>();
    config.lidar_topic = ros.at("points_topic").get<std::string>();
    config.acc_scale = ros.value("acc_scale", 0.0);
    if (!std::isfinite(config.acc_scale) || config.acc_scale < 0 || config.odom_frame.empty())
        throw std::runtime_error("Invalid GLIM acceleration scale or odom frame");
    return config;
}
inline void validate_localization(const std::string& environment, const std::string& source) {
    if (environment != "sim" && environment != "real")
        throw std::runtime_error("environment must be sim or real");
    if (source != "glim" && source != "simulator")
        throw std::runtime_error("Select navigation-localization-source explicitly: glim or simulator (no auto)");
    if (environment == "real" && source == "simulator")
        throw std::runtime_error("Ground-truth localization is only available in sim, not real");
}
} // namespace g1

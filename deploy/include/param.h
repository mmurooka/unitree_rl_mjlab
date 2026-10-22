// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <memory>
#include <iomanip>

/* ---------- logger ---------- */
namespace spdlog
{
inline void create_logger(std::string log_path)
{
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_path, 5 * 1024 * 1024, 5);

    std::vector<spdlog::sink_ptr> sinks {console_sink, rotating_sink};
    auto logger = std::make_shared<spdlog::logger>("unitree", sinks.begin(), sinks.end());

    logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%^%l%$] %v");
    logger->flush_on(spdlog::level::info);

    spdlog::set_default_logger(logger);
}

} // namespace spdlog


namespace param
{
inline std::string VERSION = "1.0.0.1";
inline std::filesystem::path bin_path;
inline std::filesystem::path proj_dir;
inline std::filesystem::path config_dir;
inline YAML::Node config;
inline std::string navigation_localization_source;
inline std::string environment = "real";
inline std::string glim_config_path;
inline std::string sensor_model_path;
inline int domain_id = 0;

inline std::filesystem::path get_bin_path() {
    std::vector<char> path(1024);
    ssize_t len = readlink("/proc/self/exe", &path[0], path.size());
    if (len != -1) {
        path[len] = '\0';  // Null-terminate the result
        return std::filesystem::path(&path[0]);
    } else {
        spdlog::error("Failed to get executable path.");
        exit(1);
    }
}

/* ---------- config.yaml ---------- */
inline void load_config_file()
{
    assert(std::filesystem::exists(bin_path)); // run param::helper before this function
    const auto executable_dir = bin_path.parent_path();
    const auto local_config = executable_dir / "config" / "config.yaml";
    const auto parent_config =
        executable_dir.parent_path() / "config" / "config.yaml";

    if (std::filesystem::exists(local_config)) {
        proj_dir = executable_dir;
    } else if (std::filesystem::exists(parent_config)) {
        // Discover configuration independently of the build directory name.
        proj_dir = executable_dir.parent_path();
    } else {
        spdlog::critical(
            "Could not find config/config.yaml next to {} or its parent.",
            executable_dir.string()
        );
        exit(1);
    }
    config_dir = proj_dir / "config";

    try {
        std::string config_file = (config_dir / "config.yaml").string();
        config = YAML::LoadFile(config_file);
    } catch (const YAML::BadFile& e) {
        spdlog::error("Failed to load config.yaml: {}", e.what());
        exit(1);
    }
}

inline std::filesystem::path parser_policy_dir(std::filesystem::path policy_dir)
{
    // Load Policy
    if (policy_dir.is_relative()) {
        policy_dir = param::proj_dir / policy_dir;
    }

    // If there is no `exported` folder in this folder,
    // then sort all the folders under this folder and take the last folder
    if (!std::filesystem::exists(policy_dir / "exported")) {
        auto dirs = std::filesystem::directory_iterator(policy_dir);
        std::vector<std::filesystem::path> dir_list;
        for (const auto& entry : dirs) {
            if (entry.is_directory()) {
                dir_list.push_back(entry.path());
            }
        }
        if (!dir_list.empty()) {
            std::sort(dir_list.begin(), dir_list.end());
            // Check if there is an `exported` folder starting from the last folder
            for (auto it = dir_list.rbegin(); it != dir_list.rend(); ++it) {
                if (std::filesystem::exists(*it / "exported")) {
                    policy_dir = *it;
                    break;
                }
            }
        }
    }
    spdlog::info("Policy directory: {}", policy_dir.string());
    return policy_dir;
}

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    bin_path = get_bin_path();
    load_config_file();

    po::options_description desc("Unitree Controller");
    desc.add_options()
        ("help,h", "produce help message")
        ("version,v", "show version")
        ("log", "record log file")
        ("network,n", po::value<std::string>()->default_value(""), "dds network interface")
        ("environment", po::value<std::string>(&environment)->default_value("real"), "sim or real")
        ("domain-id", po::value<int>(&domain_id)->default_value(0), "Unitree DDS domain (not ROS_DOMAIN_ID)")
        ("glim-config", po::value<std::string>(&glim_config_path), "external GLIM config_path directory (read only)")
        ("sensor-model", po::value<std::string>(&sensor_model_path), "model-derived sensor kinematics YAML")
        ("motion-endpoint", po::value<std::string>(), "override the OnlineMimic ZMQ endpoint")
        ("navigation-endpoint", po::value<std::string>(), "override the Navigation ZMQ endpoint")
        ("navigation-localization-source",
         po::value<std::string>(&navigation_localization_source),
         "required Navigation localization source: glim or simulator")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }
    if (vm.count("version"))
    {
        std::cout << "Version: " << VERSION << std::endl;
        exit(0);
    }

    if (environment != "sim" && environment != "real")
        throw std::runtime_error("--environment must be sim or real");
    if (domain_id < 0) throw std::runtime_error("--domain-id must be nonnegative");
    if (config["FSM"]["_"]["Navigation"]) {
        if (navigation_localization_source != "glim" && navigation_localization_source != "simulator")
            throw std::runtime_error("Specify --navigation-localization-source glim or simulator; auto is removed");
        if (environment == "real" && navigation_localization_source == "simulator")
            throw std::runtime_error("Ground-truth localization cannot be used with --environment real");
        if (navigation_localization_source == "glim" && glim_config_path.empty())
            throw std::runtime_error("Specify --glim-config with the same config_path directory used by external GLIM");
    }
    if (sensor_model_path.empty())
        sensor_model_path = (proj_dir / "../../../config/g1_sensor_frames.yaml").lexically_normal().string();
    if (vm.count("motion-endpoint"))
        config["FSM"]["OnlineMimic"]["endpoint"] = vm["motion-endpoint"].as<std::string>();
    if (vm.count("navigation-endpoint"))
        config["FSM"]["Navigation"]["endpoint"] = vm["navigation-endpoint"].as<std::string>();

#ifndef NDEBUG
    spdlog::set_level(spdlog::level::debug);
#else
    spdlog::set_level(spdlog::level::info);
#endif
    if(vm.count("log"))
    {
        std::filesystem::create_directories(proj_dir / "log");
        spdlog::create_logger(proj_dir.string() + "/log/log.txt");
    }

    return vm;
}

}

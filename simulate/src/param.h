#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    bool enable_mid360 = false;
    std::filesystem::path mid360_scan_pattern;
    std::string mid360_frame_id = "livox_frame";
    std::string mid360_lidar_topic = "/livox/lidar";
    std::string mid360_imu_topic = "/livox/imu";
    double mid360_point_rate = 200000.0;
    double mid360_publish_frequency = 10.0;
    double mid360_imu_frequency = 200.0;
    double mid360_min_range = 0.1;
    double mid360_max_range = 40.0;
    int mid360_downsample = 1;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();

            if (const auto lidar = cfg["mid360"])
            {
                enable_mid360 = lidar["enabled"].as<bool>(false);
                mid360_scan_pattern = lidar["scan_pattern"].as<std::string>("");
                mid360_frame_id = lidar["frame_id"].as<std::string>(mid360_frame_id);
                mid360_lidar_topic = lidar["lidar_topic"].as<std::string>(mid360_lidar_topic);
                mid360_imu_topic = lidar["imu_topic"].as<std::string>(mid360_imu_topic);
                mid360_point_rate = lidar["point_rate"].as<double>(mid360_point_rate);
                mid360_publish_frequency = lidar["publish_frequency"].as<double>(mid360_publish_frequency);
                mid360_imu_frequency = lidar["imu_frequency"].as<double>(mid360_imu_frequency);
                mid360_min_range = lidar["min_range"].as<double>(mid360_min_range);
                mid360_max_range = lidar["max_range"].as<double>(mid360_max_range);
                mid360_downsample = lidar["downsample"].as<int>(mid360_downsample);
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}

// Standalone test probe: no DDS, ROS, policies, or motor publisher.
#include "SensorFrames.h"
#include "glim_config.h"
#include <iostream>
#include <iomanip>

int main(int argc, char** argv) {
    try {
        if (argc == 4 && std::string(argv[1]) == "--validate") {
            g1::validate_localization(argv[2], argv[3]);
            return 0;
        }
        if (argc == 3 && std::string(argv[1]) == "--config") {
            auto c = g1::load_glim_config(argv[2]);
            std::cout << c.imu_frame << ' ' << c.odom_frame << ' '
                      << c.lidar_to_imu.translation().transpose() << '\n';
            return 0;
        }
        sensor_frames::PoseHistory poses;
        poses.add({1,1,2,3.1}); poses.add({1.2,1.2,2.2,-3.1});
        sensor_frames::PlanarPose middle;
        if (!poses.at(1.1,middle) || std::abs(std::abs(middle.yaw)-M_PI)>1e-10 ||
            poses.at(1.3,middle) || poses.at(.9,middle)) throw std::runtime_error("pose time interpolation failed");
        poses.add({2,2,2,0});
        if (poses.at(1.5,middle)) throw std::runtime_error("pose gap accepted");
        if (!poses.add({0,0,0,0}) || poses.at(1,middle)) throw std::runtime_error("pose epoch not cleared");
        // Image: root (2,3,+90deg), target one metre forward, +90deg local yaw.
        // Start: root (2,4,180deg). The target is at the start root with zero local yaw.
        auto goal=sensor_frames::rebase_goal({0,1,0,M_PI/2},{0,2,3,M_PI/2},{1,2,4,M_PI});
        if (std::hypot(goal.x,goal.y)>1e-10 || std::abs(goal.yaw)>1e-10)
            throw std::runtime_error("observation-to-start rebase failed");
        sensor_frames::History history;
        sensor_frames::State a{1.0, {}, Eigen::Quaterniond::Identity()}, b=a, out;
        b.stamp=1.02; b.q[12]=.4;
        b.orientation = Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitZ());
        history.add(a); history.add(b);
        if (!history.at(1.01,out) || std::abs(out.q[12]-.2)>1e-10 ||
            std::abs(Eigen::AngleAxisd(out.orientation).angle()-.2)>1e-10 ||
            history.at(1.5,out)) throw std::runtime_error("history interpolation/freshness failure");
        b.stamp=2; history.add(b);
        if (history.at(1.5,out)) throw std::runtime_error("history gap not rejected");
        a.stamp=0;
        if (!history.add(a) || history.at(1,out)) throw std::runtime_error("history rewind failure");
        sensor_frames::Kinematics fk(argv[1]);
        std::cout << std::setprecision(17);
        sensor_frames::Joints q;
        while (std::cin >> q[0]) {
            for (size_t i=1;i<q.size();++i) std::cin >> q[i];
            for (bool camera : {false,true}) {
                auto p = fk.root_to_sensor(q,camera);
                for (int i=0;i<4;++i) for(int j=0;j<4;++j) std::cout << p.matrix()(i,j) << ' ';
            }
            std::cout << '\n';
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n'; return 2;
    }
}

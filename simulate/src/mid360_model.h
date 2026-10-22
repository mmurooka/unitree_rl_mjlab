#pragma once

#include <mujoco/mujoco.h>
#include "glim_config.h"
#include "param.h"

// Configure only model constants, never measured/true robot or object poses.
inline void ConfigureMid360Model(mjModel* model, const std::string& directory) {
    const auto config = g1::load_glim_config(directory);
    const int lidar = mj_name2id(model, mjOBJ_SITE, "livox_frame");
    const int imu = mj_name2id(model, mjOBJ_SITE, "livox_imu");
    if (lidar < 0 || imu < 0 || model->site_bodyid[lidar] != model->site_bodyid[imu])
        throw std::runtime_error("MID-360 model requires separate lidar/IMU sites on the same body");
    const auto* q = model->site_quat + 4*lidar;
    const auto* p = model->site_pos + 3*lidar;
    Eigen::Isometry3d parent_lidar = Eigen::Isometry3d::Identity();
    parent_lidar.linear() = Eigen::Quaterniond(q[0],q[1],q[2],q[3]).toRotationMatrix();
    parent_lidar.translation() = Eigen::Vector3d(p[0],p[1],p[2]);
    const Eigen::Isometry3d parent_imu = parent_lidar * config.lidar_to_imu;
    const Eigen::Quaterniond parent_imu_q(parent_imu.linear());
    // MJCF compilation may cache body/site alignment. The new extrinsic can
    // invalidate that shortcut, including when the original site was identity.
    model->site_sameframe[imu] = mjSAMEFRAME_NONE;
    for (int i=0;i<3;++i) model->site_pos[3*imu+i] = parent_imu.translation()[i];
    model->site_quat[4*imu] = parent_imu_q.w();
    model->site_quat[4*imu+1] = parent_imu_q.x();
    model->site_quat[4*imu+2] = parent_imu_q.y();
    model->site_quat[4*imu+3] = parent_imu_q.z();
    param::config.mid360_frame_id = config.lidar_frame;
    param::config.mid360_imu_frame_id = config.imu_frame;
    param::config.mid360_lidar_topic = config.lidar_topic;
    param::config.mid360_imu_topic = config.imu_topic;
    std::cout << "MID-360 matches external GLIM config " << directory
              << "; raw acceleration in g, T_lidar_imu xyz = "
              << config.lidar_to_imu.translation().transpose() << std::endl;
}

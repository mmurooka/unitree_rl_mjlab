// Independent fixed-body sensor test; no ROS, DDS or motor publisher.
#include "mid360_model.h"
#include <iostream>

int main(int argc, char** argv) {
    char error[1024];
    auto* m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
    if (!m) { std::cerr << error; return 2; }
    ConfigureMid360Model(m, argv[2]);
    auto* d = mj_makeData(m);
    mj_forward(m,d);
    int imu=mj_name2id(m,mjOBJ_SITE,"livox_imu");
    auto cfg=g1::load_glim_config(argv[2]);
    // Fixture lidar is inverted about Y. Compare model pose and sensor output.
    Eigen::Matrix3d inverted=Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Vector3d expected=inverted*cfg.lidar_to_imu.translation();
    for(int i=0;i<3;++i) if(std::abs(d->site_xpos[3*imu+i]-expected[i])>1e-10) { std::cerr << "site actual=" << d->site_xpos[3*imu+i] << " expected=" << expected[i] << " sameframe=" << int(m->site_sameframe[imu]) << "\n"; return 3; }
    Eigen::Vector3d accel=cfg.lidar_to_imu.linear().transpose()*inverted.transpose()*Eigen::Vector3d(0,0,9.81);
    for(int i=0;i<3;++i) if(std::abs(d->sensordata[i]-accel[i])>1e-10) return 4;
    std::cout << "Configured IMU position and inverted acceleration: OK\n";
    mj_deleteData(d); mj_deleteModel(m);
}

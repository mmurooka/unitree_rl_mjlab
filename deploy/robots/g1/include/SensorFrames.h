#pragma once

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <array>
#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

// T_A_B maps B into A. All geometry is fixed model data shared with Python.
namespace sensor_frames {
using Pose = Eigen::Isometry3d;
using Joints = std::array<double, 29>;

inline Eigen::Vector3d vector3(const YAML::Node& n) {
    if (!n.IsSequence() || n.size() != 3) throw std::runtime_error("expected xyz");
    Eigen::Vector3d v(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
    if (!v.allFinite()) throw std::runtime_error("nonfinite xyz");
    return v;
}
inline Pose fixed_pose(const YAML::Node& n) {
    if (!n.IsSequence() || n.size() != 4) throw std::runtime_error("expected 4x4 pose");
    Pose p = Pose::Identity();
    for (int i = 0; i < 4; ++i) {
        if (n[i].size() != 4) throw std::runtime_error("expected 4x4 pose");
        for (int j = 0; j < 4; ++j) p.matrix()(i,j) = n[i][j].as<double>();
    }
    if (!p.matrix().allFinite() ||
        !(p.linear().transpose()*p.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-8) ||
        std::abs(p.linear().determinant()-1) > 1e-8 ||
        !p.matrix().row(3).isApprox(Eigen::RowVector4d(0,0,0,1), 1e-8))
        throw std::runtime_error("invalid rigid pose");
    return p;
}
inline double heading(const Pose& p) {
    if (p.linear().col(0).head<2>().norm() < 1e-6)
        throw std::runtime_error("root forward axis has no horizontal projection");
    return std::atan2(p.linear()(1,0), p.linear()(0,0));
}

class Kinematics {
    struct Hinge { Pose fixed; Eigen::Vector3d axis, pivot; int sdk; };
    std::vector<Hinge> hinges;
    Pose torso_lidar, torso_camera;
public:
    explicit Kinematics(const std::string& path) {
        auto doc = YAML::LoadFile(path);
        if (doc["schema_version"].as<int>() != 1 || doc["root_frame"].as<std::string>() != "pelvis")
            throw std::runtime_error("unsupported sensor model schema/root");
        for (auto j : doc["waist"]) {
            Hinge h;
            h.sdk = j["sdk_index"].as<int>();
            if (h.sdk < 0 || h.sdk >= 29) throw std::runtime_error("invalid SDK index");
            h.fixed = Pose::Identity();
            h.fixed.translation() = vector3(j["translation"]);
            auto q = j["quaternion_wxyz"].as<std::vector<double>>();
            if (q.size() != 4) throw std::runtime_error("expected wxyz quaternion");
            Eigen::Quaterniond quat(q[0],q[1],q[2],q[3]);
            if (!quat.coeffs().allFinite() || quat.norm() < 1e-9)
                throw std::runtime_error("invalid model quaternion");
            h.fixed.linear() = quat.normalized().toRotationMatrix();
            h.axis = vector3(j["axis"]);
            if (h.axis.norm() < 1e-9) throw std::runtime_error("invalid joint axis");
            h.axis.normalize();
            h.pivot = vector3(j["pivot"]);
            hinges.push_back(h);
        }
        if (hinges.size() != 3) throw std::runtime_error("expected three waist hinges");
        torso_lidar = fixed_pose(doc["torso_to_lidar"]);
        torso_camera = fixed_pose(doc["torso_to_color_optical"]);
    }
    Pose root_to_sensor(const Joints& measured, bool camera = false) const {
        for (auto q : measured) if (!std::isfinite(q)) throw std::runtime_error("nonfinite measured angle");
        Pose p = Pose::Identity();
        for (const auto& h : hinges) {
            Pose joint = Pose::Identity();
            joint.linear() = Eigen::AngleAxisd(measured[h.sdk], h.axis).toRotationMatrix();
            joint.translation() = h.pivot - joint.linear()*h.pivot;
            p = p * h.fixed * joint;
        }
        return p * (camera ? torso_camera : torso_lidar);
    }
};

struct State {
    double stamp;
    Joints q;
    Eigen::Quaterniond orientation;
};
class History {
    mutable std::mutex mutex;
    std::deque<State> states;
public:
    // Returns true on a timestamp rewind; caller must invalidate active odometry.
    bool add(State s) {
        if (!std::isfinite(s.stamp) || !s.orientation.coeffs().allFinite() || s.orientation.norm() < 1e-9)
            throw std::runtime_error("invalid measured state");
        for (auto q : s.q) if (!std::isfinite(q)) throw std::runtime_error("invalid joint angle");
        s.orientation.normalize();
        std::lock_guard<std::mutex> lock(mutex);
        const bool rewind = !states.empty() && s.stamp < states.back().stamp;
        if (rewind) states.clear();
        if (!states.empty() && s.stamp == states.back().stamp) states.back() = s;
        else states.push_back(s);
        while (states.size() > 1 && states.front().stamp < s.stamp-5) states.pop_front();
        return rewind;
    }
    bool at(double stamp, State& out, double tolerance=.03) const {
        std::lock_guard<std::mutex> lock(mutex);
        if (!std::isfinite(stamp) || states.empty() || stamp < states.front().stamp-tolerance ||
            stamp > states.back().stamp+tolerance) return false;
        if (stamp <= states.front().stamp) { out = states.front(); return true; }
        for (size_t i=1; i<states.size(); ++i) {
            const auto& a = states[i-1]; const auto& b = states[i];
            if (stamp == b.stamp) { out=b; return true; }
            if (stamp < b.stamp) {
                if (b.stamp-a.stamp > 2*tolerance) return false;
                double t = (stamp-a.stamp)/(b.stamp-a.stamp);
                out.stamp = stamp;
                for (size_t j=0;j<29;++j) out.q[j] = (1-t)*a.q[j]+t*b.q[j];
                out.orientation = a.orientation.slerp(t,b.orientation);
                return true;
            }
        }
        out=states.back(); return true;
    }
    double newest_stamp() const {
        std::lock_guard<std::mutex> lock(mutex);
        return states.empty() ? -INFINITY : states.back().stamp;
    }
};
} // namespace sensor_frames

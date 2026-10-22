#pragma once
#include <eigen3/Eigen/Core>
#include <cmath>
#include <utility>

namespace g1 {
// Shared path geometry for controller adoption and passive pre-execution preview.
// Timing/feedback belong to Navigation; preview is not a physics prediction.
inline std::pair<Eigen::Vector2f, float> navigation_curve_point(
    const Eigen::Vector2f& goal, float yaw, float tangent_scale, float u)
{
    yaw = std::atan2(std::sin(yaw), std::cos(yaw));
    if (u <= 0) return {Eigen::Vector2f::Zero(), 0.0f};
    if (u >= 1) return {goal, yaw};
    const float sign = goal.x() < 0 ? -1.0f : 1.0f;
    const float length = sign*tangent_scale*goal.norm();
    const Eigen::Vector2f a(length,0), b = goal-length*Eigen::Vector2f(std::cos(yaw),std::sin(yaw));
    const float v=1-u;
    const Eigen::Vector2f position = 3*v*v*u*a + 3*v*u*u*b + u*u*u*goal;
    const Eigen::Vector2f tangent = 3*v*v*a + 6*v*u*(b-a) + 3*u*u*(goal-b);
    const float heading = tangent.squaredNorm()>1e-10f
        ? std::atan2(sign*tangent.y(),sign*tangent.x()) : u*yaw;
    return {position,heading};
}
}

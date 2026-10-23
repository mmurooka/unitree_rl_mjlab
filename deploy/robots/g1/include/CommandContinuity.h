#pragma once
#include <cmath>
#include <string>
#include <vector>
#include <fmt/format.h>

// Both inputs are motor POSITION COMMANDS in the same SDK joint order.
// A measured-to-command tracking error is deliberately not a continuity test.
inline std::string command_continuity_error(const std::vector<float>& before,
                                           const std::vector<float>& after,
                                           float threshold, float leg_threshold) {
    if (before.size()!=29 || after.size()!=29)
        return "ERROR invalid command vector size for continuity check";
    if (!std::isfinite(threshold) || threshold <= 0 ||
        !std::isfinite(leg_threshold) || leg_threshold <= 0)
        return "ERROR invalid command continuity threshold";
    float maximum=0, worst_ratio=0, limit=threshold; size_t joint=0;
    for (size_t i=0;i<before.size();++i) {
        if (!std::isfinite(before[i]) || !std::isfinite(after[i]))
            return "ERROR nonfinite motor command for continuity check";
        // SDK 0..11 are the legs; 12..28 are waist and arms.
        const float joint_limit = i < 12 ? leg_threshold : threshold;
        const float change = std::abs(before[i]-after[i]);
        if (change/joint_limit > worst_ratio) {
            worst_ratio=change/joint_limit; maximum=change; joint=i; limit=joint_limit;
        }
    }
    if (worst_ratio>1.0f)
        return fmt::format("ERROR command-to-command start discontinuity at joint {}: {:.2f} deg (limit {:.2f} deg)",
                           joint, maximum*180/M_PI, limit*180/M_PI);
    return {};
}

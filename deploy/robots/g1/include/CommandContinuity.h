#pragma once
#include <cmath>
#include <string>
#include <vector>
#include <fmt/format.h>

// Both inputs are motor POSITION COMMANDS in the same SDK joint order.
// A measured-to-command tracking error is deliberately not a continuity test.
inline std::string command_continuity_error(const std::vector<float>& before,
                                           const std::vector<float>& after,
                                           float threshold) {
    if (before.size()!=29 || after.size()!=29)
        return "ERROR invalid command vector size for continuity check";
    float maximum=0; size_t joint=0;
    for (size_t i=0;i<before.size();++i) {
        if (!std::isfinite(before[i]) || !std::isfinite(after[i]))
            return "ERROR nonfinite motor command for continuity check";
        if (std::abs(before[i]-after[i])>maximum) {
            maximum=std::abs(before[i]-after[i]); joint=i;
        }
    }
    if (maximum>threshold)
        return fmt::format("ERROR command-to-command start discontinuity at joint {}: {:.2f} deg (limit {:.2f} deg)",
                           joint, maximum*180/M_PI, threshold*180/M_PI);
    return {};
}

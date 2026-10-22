#include "CommandContinuity.h"
#include <limits>
#include <iostream>
int main() {
    std::vector<float> before(29,.7f), after=before;
    // Low-gain measurements can differ by much more than the limit; no measured
    // angle participates in this API. Identical commands must pass.
    if (!command_continuity_error(before,after,.2f).empty()) return 1;
    after[18] += .3f;
    auto error=command_continuity_error(before,after,.2f);
    if (error.find("joint 18")==std::string::npos || error.find("command-to-command")==std::string::npos) return 2;
    after[18]=std::numeric_limits<float>::quiet_NaN();
    if (command_continuity_error(before,after,.2f).empty()) return 3;
    if (command_continuity_error({},before,.2f).empty()) return 4;
    std::cout << "Command continuity checks passed\n";
}

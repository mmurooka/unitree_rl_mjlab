#include "CommandContinuity.h"
#include <limits>
#include <iostream>
int main() {
    std::vector<float> before(29,.7f), after=before;
    // Low-gain measurements can differ by much more than the limit; no measured
    // angle participates in this API. Identical commands must pass.
    if (!command_continuity_error(before,after,.2f,.3f).empty()) return 1;
    after[18] += .3f;
    auto error=command_continuity_error(before,after,.2f,.3f);
    if (error.find("joint 18")==std::string::npos || error.find("command-to-command")==std::string::npos) return 2;
    after[18]=std::numeric_limits<float>::quiet_NaN();
    if (command_continuity_error(before,after,.2f,.3f).empty()) return 3;
    if (command_continuity_error({},before,.2f,.3f).empty()) return 4;
    before.assign(29, 0); after=before;
    const float upper=20*M_PI/180, legs=30*M_PI/180;
    for (int i=0;i<29;++i) {
        after=before; after[i]=25*M_PI/180;
        if (command_continuity_error(before,after,upper,legs).empty() != (i<12)) return 5;
        after[i]=31*M_PI/180;
        if (command_continuity_error(before,after,upper,legs).empty()) return 6;
    }
    // A larger allowed leg change must not hide a smaller rejected arm change.
    after=before; after[6]=29*M_PI/180; after[15]=21*M_PI/180;
    auto mixed=command_continuity_error(before,after,upper,legs);
    if (mixed.find("joint 15")==std::string::npos || mixed.find("limit 20.00")==std::string::npos) return 7;
    after=before; after[11]=legs; after[12]=upper;
    if (!command_continuity_error(before,after,upper,legs).empty()) return 8;
    if (command_continuity_error(before,after,upper,0).empty()) return 9;
    std::cout << "Command continuity checks passed\n";
}

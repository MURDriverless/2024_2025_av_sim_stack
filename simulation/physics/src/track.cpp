#include "track.h"



//at this stage, we only assume the straight track enviornment given
Track::Track(double roadLength, double coneSpacing) {
    for (double i = 0; i <= roadLength; i += coneSpacing) {
        leftCones.push_back(Cone(i, 0.5, "yellow"));
        rightCones.push_back(Cone(i, -0.5, "blue")); // assume the bike is at 0
    }
}

std::vector<Cone> Track::getAllCones() const {
    std::vector<Cone> allCones = leftCones;
    allCones.insert(allCones.end(), rightCones.begin(), rightCones.end());
    return allCones;
}

void Track::reportTrackStatus() {
    std::cout << "Left side cones:\n";
    for (const auto& cone : leftCones) {
        cone.reportConePosition();
    }

    std::cout << "\nRight side cones:\n";
    for (const auto& cone : rightCones) {
        cone.reportConePosition();
    }
}

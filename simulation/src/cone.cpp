#include "cone.h"


Cone::Cone(double xPos, double yPos, std::string coneColour) : x(xPos), y(yPos), colour(coneColour) {}

void Cone::reportConePosition() {
    std::cout << "Cone position: (" << x << ", " << y << "), Colour: " << colour << "\n";
}
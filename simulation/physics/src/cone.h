#ifndef CONE_H
#define CONE_H

#include <iostream>
#include <vector>
#include <string>

class Cone {
public:
    double x, y;
    std::string colour;
    

    Cone(double xPos, double yPos, std::string coneColour);

    void reportConePosition();
};

#endif
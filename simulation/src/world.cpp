#include <iostream>
#include <cmath>
#include <vector>
#include "track.h"
#include "bike.h"

class World {
private:
    Track track;  
    Bike bicycle;  
    double fov_horizontal_degrees; // The width of the visible area in degrees, from the leftmost to the rightmost points.

public:
    World(double roadLength, double coneSpacing, double fovDegrees)
        : track(roadLength, coneSpacing), bicycle(), fov_horizontal_degrees(fovDegrees) {}

    std::vector<Cone> cone_info_at_given_pose_and_fov() {
        std::vector<Cone> visibleCones;
        double px = bicycle.getX();
        double py = bicycle.getY();
        double theta = bicycle.getTheta();
        
        double half_fov = 0.5 * fov_horizontal_degrees * (M_PI / 180.0);  // in radians and 0.5m away from the cones 

        auto allCones = track.getAllCones();

        for (const auto& cone : allCones) {
            double deltaX = cone.x - px;
            double deltaY = cone.y - py;
            double distance = sqrt(deltaX * deltaX + deltaY * deltaY); 
            double angleToCone = atan2(deltaY, deltaX) - theta; //the angle of the vector from the bike to the cone. And off the heading angle by - theta

            if (angleToCone > M_PI) angleToCone -= 2 * M_PI;
            if (angleToCone < -M_PI) angleToCone += 2 * M_PI; //this help the angle is withing -180 to 180

            // filtering and fabs() function in C++ is used to return the absolute value of an argument passed to the function. 
            if (fabs(angleToCone) <= half_fov && distance <= 1.0) {  // 1.0 becauz ouster lader clear view range is 1.0m
                visibleCones.push_back(cone);
            }
        }

        return visibleCones;
    }

    void updateWorld(double timeStep) {
        bicycle.updateState();  /

        auto visibleCones = cone_info_at_given_pose_and_fov();
        std::cout << "Visible Cones: \n";
        for (const auto& cone : visibleCones) {
            cone.reportConePosition();
        }
    }

    void setBikeInput(double F, double Ddelta) {
        bicycle.setInput(F, Ddelta);
    }

    void reportBikeState() {
        bicycle.reportState();
    }
};

//testing
int main() {
    World world(10.0, 1.0, 80.0);  // track;ength, cone range, fov degree
    for (int i = 0; i < 10; ++i) {
        std::cout << "Simulation step " << i+1 << ":\n";
        world.setBikeInput(1, 0);  // straigt
        world.updateWorld(0.1);    // 0.1sec updating
        world.reportBikeState();
    }

    return 0;
}



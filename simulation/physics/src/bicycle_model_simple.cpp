#include <iostream>
#include <cmath>

// Constants
const double PI = 3.14;
const double DT = 0.1; // Time step in seconds
const double VELOCITY = 1.0; // Constant velocity in meters per second
const double STEERING_ANGLE = 30 * (PI / 180); // Steering angle in radians 
const double WHEELBASE = 2.0; // Distance between the front and rear wheels in meters

// Function to update the position and orientation of the bicycle
void updatePosition(double &x, double &y, double &theta, double velocity, double steering_angle, double dt) {
    // Bicycle model equations
    x += velocity * cos(theta) * dt;
    y += velocity * sin(theta) * dt;
    theta += (velocity / WHEELBASE) * tan(steering_angle) * dt;
}

int main() {
    // Initial position and orientation of the circle
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0; // Initial orientation in radians

    // Move forward for 10 meters
    double distance_traveled = 0.0;
    while (distance_traveled < 10.0) {
        updatePosition(x, y, theta, VELOCITY, 0.0, DT);
        distance_traveled += VELOCITY * DT;
        std::cout << "Position: (" << x << ", " << y << "), Orientation: " << theta * (180 / PI) << " degrees\n";
    }

    // Curve left for the specified distance
    distance_traveled = 0.0;
    while (distance_traveled < 10) {
        updatePosition(x, y, theta, VELOCITY, STEERING_ANGLE, DT);
        distance_traveled += VELOCITY * DT;
        std::cout << "Position: (" << x << ", " << y << "), Orientation: " << theta * (180 / PI) << " degrees\n";
    }

    return 0;
}
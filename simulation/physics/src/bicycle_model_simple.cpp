#include <iostream>
#include <cmath>
#include <vector>

// Constants
const double PI = 3.14;
const double DT = 0.1; // Time step in seconds

class Bike {

    /*
    parameters: F, Ddelta, m, Lf, Lr
    */

    private:
    std::vector<double> state = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> parameters = {0, 0, 1, 1, 1};
    double distance = 0;

    public:
    Bike();

    double getDistance() {
        return distance;
    }

    void resetDistance() {
        distance = 0;
    }

    // Getter function to access a value at a specific index
    void reportState() {
        std::cout << "Pos: (" << state[0] << ", " << state[1] << "), O: " << state[2] * (180 / PI) << " d\n";
        std::cout << "Vel: (" << state[3] << ", " << state[4] << ") d\n";
    }

        // Setter function to change a value at a specific index
    void updateState() {
        std::vector<double> state_dot;
        state_dot = computeStateDot(state, state_dot, parameters);
        for (size_t i = 0; i < state.size(); ++i) {
            state[i] += state_dot[i]*DT;
            if (state[6] > PI/3) {
                state[6] = PI/3;
                parameters[1] = 0;
            }
        }
        distance += (state_dot[0] + state_dot[1])*DT;
    }

    void setInput(double F, double Ddelta) {
        parameters[1] = F;
        parameters[2] = Ddelta;
    }

    static std::vector<double> computeStateDot(std::vector<double> state, std::vector<double> state_dot, std::vector<double> parameters) {
        double F_on_m = parameters[0]/parameters[2];
        double delta_dot_sec2_delta_vx_plus_tan_delta_v_dot = parameters[1]*(1.0+tan(state[6])*tan(state[6]))*state[3] + tan(state[6])*F_on_m;
        state_dot[0] = state[3] * cos(state[2]) - state[4] * sin(state[2]);
        state_dot[1] = state[3] * sin(state[2]) + state[4] * cos(state[2]);
        state_dot[2] = state[5];
        state_dot[3] = F_on_m;
        state_dot[4] = ( delta_dot_sec2_delta_vx_plus_tan_delta_v_dot ) * parameters[4] / (parameters[3]+parameters[4]);
        state_dot[5] = ( delta_dot_sec2_delta_vx_plus_tan_delta_v_dot ) * 1.0      / (parameters[3]+parameters[4]);
        state_dot[6] = parameters[1];
        return state_dot;
    }
};


// Function to update the position and orientation of the bicycle
/*
void updatePosition(double &x, double &y, double &theta, double velocity, double steering_angle, double dt) {
    // Bicycle model equations
    x += velocity * cos(theta) * dt;
    y += velocity * sin(theta) * dt;
    theta += (velocity / WHEELBASE) * tan(steering_angle) * dt;
}
*/

int main() {
    // Initial position and orientation of the circle
    Bike car;

    while (car.getDistance() < 10.0) {
        car.setInput(1,0);
        car.updateState();
        car.reportState();
    }

    car.resetDistance();

    // Curve left for the specified distance
    while (car.getDistance() < 10) {
        car.setInput(1,1);
        car.updateState();
        car.reportState();
    }

    return 0;
}
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
    std::vector<double> state_dot = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> parameters = {0, 0, 1, 1, 1};
    double distance = 0;

    void computeStateDot() {
        double F_on_m = parameters[0]/parameters[2];
        double delta_dot_sec2_delta_vx_plus_tan_delta_v_dot = parameters[1]*(1.0+tan(state[6])*tan(state[6]))*state[3] + tan(state[6])*F_on_m;
        state_dot[0] = state[3] * cos(state[2]) - state[4] * sin(state[2]);
        state_dot[1] = state[3] * sin(state[2]) + state[4] * cos(state[2]);
        state_dot[2] = state[5];
        state_dot[3] = F_on_m;
        state_dot[4] = ( delta_dot_sec2_delta_vx_plus_tan_delta_v_dot ) * parameters[4] / (parameters[3]+parameters[4]);
        state_dot[5] = ( delta_dot_sec2_delta_vx_plus_tan_delta_v_dot ) * 1.0      / (parameters[3]+parameters[4]);
        state_dot[6] = parameters[1];
    }

    public:

    double getDistance() {
        return distance;
    }

    void resetDistance() {
        distance = 0;
    }

    // Getter function to access a value at a specific index
    void reportState() {
        std::cout << "Distance: " << distance << " m\n";
        std::cout << "Pos: (" << state[0] << ", " << state[1] << "), O: " << state[2] * (180 / PI) << " degrees\n";
        std::cout << "Vel: (" << state[3] << ", " << state[4] << ") d\n\n";

    }

        // Setter function to change a value at a specific index
    void updateState() {
        computeStateDot();
        for (size_t i = 0; i < state.size(); ++i) {
            state[i] += state_dot[i]*DT;
            if (state[6] > PI/3) {
                state[6] = PI/3;
                parameters[1] = 0;
            }
        }
        distance += (fabs(state[0]) + fabs(state[1]))*DT;
    }

    void setInput(double F, double Ddelta) {
        parameters[0] = F;
        parameters[1] = Ddelta;
    }
};

int main() {
    // Initial position and orientation of the circle
    Bike car;

    car.setInput(1,0);
    while (car.getDistance() < 10.0) {
        car.updateState();
        car.reportState();
    }

    std::cout << "FINISH\n";
    car.resetDistance();

    // Curve left for the specified distance
    car.setInput(1,0.2);
    while (car.getDistance() < 10) {
        car.updateState();
        car.reportState();
    }

    return 0;
}
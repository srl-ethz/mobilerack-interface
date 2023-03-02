// Copyright 2018 ...
#include "mobilerack-interface/ValveController.h"
#include <cmath>
#include <iostream>
/**
 * @file example_ValveController_sinusoidal.cpp
 * @brief sends a sinusoidal wave to 3 actuators (or multiples of 3), where each phase is offset by 2PI/3 to create rotational motion.
*/
const double duration = 20;
const double timestep = 0.01;

int sinusoid(double phase) { return 150 + 150 * sin(phase); }

int main() {
    std::vector<int> map = {3,2,7,1,5,6};
    int num_segments = 2;
    ValveController vc{"192.168.0.100", map, 400};
    double phase;
    double coef = 0.;  // moves between 0 and 1 to gradually increase and decrease pressure at start and end
    srl::Rate r{1. / timestep};
    for (double time = 0; time < duration; time += timestep) {
        phase = time * 2;
        if (time < 2)
            coef += timestep / 2;  // ramp up pressure
        if (time > duration - 2)
            coef -= timestep / 2;  // ramp down pressure
        if (coef > 1)
            coef = 1; // clamp to 1

        for (int segment = 0; segment < num_segments; segment ++){
            vc.setSinglePressure(0 + 3 * segment, coef * sinusoid(phase));
            vc.setSinglePressure(1 + 3 * segment, coef * sinusoid(phase + 2 * M_PI / 3));
            vc.setSinglePressure(2 + 3 * segment, coef * sinusoid(phase + 4 * M_PI / 3));
        }
        r.sleep();
    }
    return 1;
}

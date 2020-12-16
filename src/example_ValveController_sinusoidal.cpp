// Copyright 2018 ...
#include "ValveController.h"
#include <cmath>
#include <iostream>
/**
 * @file example_ValveController_sinusoidal.cpp
 * @brief sends a sinusoidal wave to 4 actuators, where each phase is offset by PI/4.
*/
const double duration = 20;
const double timestep = 0.01;

int sinusoid(double phase) { return 150 + 150 * sin(phase); }

int main() {
    ValveController vc{};
    double phase;
    for (double time = 0; time < duration; time += timestep) {
        phase = time * 2;
        vc.setSinglePressure(st_params::valve::map[0], sinusoid(phase));
        vc.setSinglePressure(st_params::valve::map[1], sinusoid(phase + PI / 2));
        vc.setSinglePressure(st_params::valve::map[2], sinusoid(phase + 2 * PI / 2));
        vc.setSinglePressure(st_params::valve::map[3], sinusoid(phase + 3 * PI / 2));
        sleep(timestep);
    }
    vc.disconnect();
    return 1;
}

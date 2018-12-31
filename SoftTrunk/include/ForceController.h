#ifndef FORCECONTROLLER_H
#define FORCECONTROLLER_H

#include "MiniPID.h"
#include "matplotlibcpp.h"
#include "MPA.h"
#include <thread>
#include <vector>
#include <iostream>
#include <thread>
#include "SoftTrunk_common_defs.h"
#include <chrono>

/**
 * @brief Implements an individual PID control for each valve of the FESTO valve array.
 * @details example_sinusoidal.cpp and example_forceController.cpp are demos of this library.
 *
 */
class ForceController {
private:
    void controllerThread();

    bool run;
    std::vector<int> commanded_pressures;
    int DoF;
    std::vector<MiniPID> pid;
    MPA mpa{VALVE_ADDRESS, "502"};
    std::thread controller_thread;
    std::vector<double> seconds_log;
    std::vector<double> pressure_log;
    std::vector<double> commandpressure_log;
    int maxPressure;
    std::chrono::high_resolution_clock::time_point logBeginTime;

public:
    void setSinglePressure(int, int);

    explicit ForceController(int, int);

    void disconnect();
};

#endif

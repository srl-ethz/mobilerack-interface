#ifndef FORCECONTROLLER_H
#define FORCECONTROLLER_H

#include "MiniPID.h"
#include "MPA.h"
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>
#include "SoftTrunk_common_defs.h"
#include <chrono>

/**
 * @brief Interface for the pressure valve array. Implements an individual PID control for each valve.
 * The PID controller runs as a separate thread from the main code.
 * @details example_sinusoidal.cpp and example_ValveController.cpp are demos of this library.
 *
 */
class ValveController {
private:
    /**
     * @brief thread on which the PID controllers run.
     */
    void controllerThread();

    /**
     * @brief thread runs as long as this is set to true
     */
    bool run;
    std::vector<int> commanded_pressures;
    int DoF;
    std::vector<MiniPID> pid;
    MPA mpa{VALVE_ADDRESS, "502"};
    std::thread controller_thread;
    std::vector<double> seconds_log;
    std::vector<std::vector<int>> pressure_log;
    std::vector<std::vector<int>> commandpressure_log;
    int maxPressure;
    std::chrono::high_resolution_clock::time_point logBeginTime;

public:
    /**
     * @brief set pressure for a single valve.
     * @param index valve id
     * @param pressure pressure, in mbar.
     */
    void setSinglePressure(int index, int pressure);

    /**
     * @param maxValveIndex maximum valve index used in current state
     * @param maxPressure set the output limit of all the valves, for that arm won't pop
     */
    explicit ValveController(int maxValveIndex, int maxPressure);

    /**
     * @brief stops the PID controller and outputs log.
     */
    void disconnect();
};

#endif

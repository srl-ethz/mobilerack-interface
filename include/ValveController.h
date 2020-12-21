#pragma once

#include "MiniPID.h"
#include "MPA.h"
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
#include "SoftTrunk_common.h"
#include <chrono>

/**
 * @brief Interface for the pressure valve array. Implements an individual PID control (optional) for each valve.
 * The PID controller runs as a separate thread from the main code.
 * @details example_sinusoidal.cpp and example_ValveController.cpp show example usage of this library.
 *
 */
class ValveController {
private:
    /**
     * @brief thread on which the PID controllers run.
     */
    void controllerThread();

    /** @brief write CSV log to log_pressure.csv */
    void write_log();

    /** @brief thread runs as long as this is set to true */
    bool run;

    /** @brief holds the desired pressure values for each actuator */
    std::vector<int> desired_pressures;
    std::vector<MiniPID> pid;
    MPA* mpa;
    std::thread controller_thread;

    // variables used to save the log data
    std::vector<double> seconds_log;
    std::vector<std::vector<int>> sensor_pressure_log;
    std::vector<std::vector<int>> desired_pressure_log;
    std::chrono::high_resolution_clock::time_point logBeginTime;

public:
    /**
     * @brief set pressure for a single valve.
     * @param index ID of actuator (not ID of valve)
     * @param pressure pressure, in mbar.
     */
    void setSinglePressure(int index, int pressure);

    explicit ValveController();

    /**
     * @brief stops the PID controller and outputs log.
     */
    void disconnect();
};

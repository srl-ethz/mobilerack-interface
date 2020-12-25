#pragma once

#include "MiniPID.h"
#include "MPA.h"
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
#include "common.h"
#include <chrono>
#include "fmt/core.h"
#include "fmt/ostream.h"

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
    MPA *mpa;
    std::thread controller_thread;

    const int num_valves_total = 16;
    const char *address;
    const std::vector<int> &map;
    const int max_pressure;
    const bool use_pid = false;
    const bool log = true;

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

    /**
     * @param address
     * @param map
     * @param max_pressure max pressure that can be sent out. Useful to prevent puncture of the arm with too high a pressure.
     * for DragonSkin 30, set to 1200.
     * for DragonSkin 10, set to 400.
     * (not throughly examined- a larger or smaller value may be better)
     */
    explicit ValveController(const char *address, const std::vector<int> &map, const int max_pressure);

    /**
     * @brief stops the PID controller and outputs log.
     */
    void disconnect();
};

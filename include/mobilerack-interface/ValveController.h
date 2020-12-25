#pragma once

#include "common.h"

#include "MiniPID.h"
#include "MPA.h"
#include <fstream>
#include "fmt/core.h"
#include "fmt/ostream.h"
#include <mutex>

/** @brief Interface for the pressure valve array. Implements an individual PID control (optional) for each valve. */
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
    std::mutex mtx;

    const int num_valves_total = 16; /** @brief total number of valves in the Festo valve array setup */
    const std::vector<int> map;
    const int max_pressure;
    const bool use_pid = false; /** @brief use PID control for pressure (kind of pointless since it's already done by valves */
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
     * @param address IP address of Festo valves.
     * @param map map[i] is the valve ID for i-th actuator.
     * @param max_pressure max pressure that can be sent out. Useful to prevent puncture of the arm with too high a pressure.
     * 1200 for DragonSkin 30, 400 for DragonSkin 10 is recommended.
     * (not throughly examined- a larger or smaller value may be better)
     */
    ValveController(const char *address, const std::vector<int> &map, const int max_pressure);

    /**
     * @brief disconnects from valve, and outputs log.
     */
    void disconnect();
};

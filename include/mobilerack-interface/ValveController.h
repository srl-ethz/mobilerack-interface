#pragma once

#include "common.h"

#include "MiniPID.h"
#include "MPA.h"
#include <fstream>
#include "fmt/core.h"
#include "fmt/ostream.h"
#include <mutex>

/** @brief Interface for the pressure valve array. */
class ValveController {
private:
    /**
     * @brief thread on which the data is sent to Valves.
     */
    void controllerThread();

    /** @brief thread runs as long as this is set to true */
    std::atomic<bool> run;

    /** @brief total number of valves in the Festo valve array setup */
    const int num_valves_total {16}; 

    /** @brief holds the desired pressure values for each actuator */
    std::vector<int> desired_pressures {num_valves_total, 0};

    // sensor_pressures and output_pressures use valve IDs for easier interfacing with mpa library
    std::vector<int> sensor_pressures {num_valves_total, 0};
    std::vector<int> output_pressures {num_valves_total, 0};

    MPA *mpa;
    std::thread controller_thread;
    std::mutex mtx;
    const double hz;

    const std::vector<int> map;
    const int max_pressure;
    const bool log = true;
    std::fstream log_file;
    
    std::chrono::high_resolution_clock::time_point logBeginTime;
public:
    /**
     * @brief set pressure for a single valve.
     * @param index ID of actuator (not ID of valve)
     * @param pressure pressure, in mbar.
     */
    void setSinglePressure(int index, int pressure);

    /**
     * @brief use external clock (like from QTM) and use that to sync the timestamps for logged pressure values.
     * "# timestamp synced\n" will be inserted into the csv at this point, to make it easy to find which point the timestamp got synced.
     * @param currentTimeSecs external clock's current time, in milliseconds
     */
    void syncTimeStamp(unsigned long int currentTimeMillis);
    /**
     * @param address IP address of Festo valves.
     * @param map map[i] is the valve ID for i-th actuator.
     * @param max_pressure max pressure that can be sent out. Useful to prevent puncture of the arm with too high a pressure.
     * 1200 for DragonSkin 30, 400 for DragonSkin 10 is recommended.
     * (not throughly examined- a larger or smaller value may be better)
     * @param hz how often to send pressure data to (and log from) valves.
     * Default is 100 Hz but that is only possible with a wired connection, with wifi it would be slowed down to around 30Hz due to latency
     */
    ValveController(const char *address, const std::vector<int> &map, const int max_pressure, double hz = 100);

    int getSinglePressure(int index);

    void setPressures(const std::vector<int>& pressures);

    std::vector<int> getPressures();

    /**
     * @brief disconnects from valve, and outputs log.
     */
    ~ValveController();
};

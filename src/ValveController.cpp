// Copyright 2018 ...
#include "mobilerack-interface/ValveController.h"

ValveController::ValveController(const char *address, const std::vector<int> &map, const int max_pressure, double hz) :
        map(map), max_pressure(max_pressure), hz(hz){
    fmt::print("connecting to valves at {}...\n", address);
    mpa = std::make_unique<MPA>(address, "502");

    desired_pressures.resize(map.size());
    measured_pressures.resize(map.size());

    sensor_pressures.resize(num_valves_total);
    output_pressures.resize(num_valves_total);

    if (!mpa->connect()) {
        fmt::print("Failed to connect to valves at {}.\n", address);
        return;
    }
    fmt::print("Successfully connected to MPA at {}, sending at {} Hz.\n", address, hz);

    run = true;
    controller_thread = std::thread(&ValveController::controllerThread, this);
}

void ValveController::setSinglePressure(int index, int pressure) {
    std::lock_guard<std::mutex> lock(mtx);
    assert(0 <= index && index < map.size());
    desired_pressures[index] = pressure;
}

int ValveController::getSinglePressure(int index) {
    std::lock_guard<std::mutex> lock(mtx);
    assert(0 <= index && index < map.size());
    return measured_pressures[index];
}

void ValveController::setPressures(const std::vector<int>& pressures) {
    std::lock_guard<std::mutex> lock(mtx);
    std::copy(pressures.begin(), pressures.end(), desired_pressures.begin());
}

std::vector<int> ValveController::getPressures() {
    std::lock_guard<std::mutex> lock(mtx);
    return measured_pressures;
}

void ValveController::syncTimeStamp(unsigned long int currentTimeMillis){
    std::lock_guard<std::mutex> lock(mtx);
    logBeginTime = std::chrono::high_resolution_clock::now() - std::chrono::milliseconds(currentTimeMillis);
    if (log)
        log_file << "# timestamp synced\n";
}

void ValveController::controllerThread() {
    logBeginTime = std::chrono::high_resolution_clock::now();   
    if (log){
        std::cout << "Outputting log of ValveController to log_pressure.csv..." << std::endl;
        log_file.open("log_pressure.csv", std::fstream::out);
        // write first line of log
        log_file << "time(sec)";
        for (int i = 0; i < map.size(); ++i) {
            log_file << fmt::format(", p_des[{}]", i); 
        }
        for (int i = 0; i < map.size(); ++i) {
            log_file << fmt::format(", p_meas[{}]", i);
        }
        log_file << "\n";
    }
   
    for (int i = 0; i < num_valves_total; i++) {
        output_pressures[i] = 0;
    }
    srl::Rate r{hz};
    while (run) {
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);
        mpa->get_all_pressures(&sensor_pressures);
        for (int i = 0; i < map.size(); i++) {
            // constrain pressure value to between 0 and max_pressure
            // valve goes haywire when it tries to write negative value
            output_pressures[map[i]] = std::max(0, std::min(desired_pressures[i], max_pressure));
            measured_pressures[i] = sensor_pressures[map[i]];
        }
        mpa->set_all_pressures(output_pressures);
        if (log) {
            log_file << (double) (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - logBeginTime).count()) / 1000.;
            for (int valve_id: map)
                log_file << fmt::format(", {}", output_pressures[valve_id]);
            for (int valve_id: map)
                log_file << fmt::format(", {}", sensor_pressures[valve_id]);
            log_file << "\n";
        }
    }

    // turn off all valves
    for (int i = 0; i < num_valves_total; i++) {
        output_pressures[i] = 0;
    }
    mpa->set_all_pressures(output_pressures);
    mpa->disconnect();
    if (log)
        log_file.close();
}

ValveController::~ValveController() {
    run = false;
    controller_thread.join();
    fmt::print("Stopped ValveController thread.\n");
}

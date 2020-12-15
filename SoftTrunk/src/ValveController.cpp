// Copyright 2018 ...
#include "ValveController.h"

ValveController::ValveController(){
    mpa = new MPA(st_params::valve::address, "502");
    desired_pressures.resize(st_params::valve::map.size());
    std::cout << "Connecting to MPA." << std::endl;
    if (!mpa->connect()) {
        std::cout << "Failed to connect to MPA." << std::endl;
        return;
    }
    std::cout << "Successfully connected to MPA." << std::endl;

    if (st_params::valve::use_pid){
        // Ziegler-Nichols method
        double Ku = 2.6;
        double Tu = 0.14;
        double KP = 0.6 * Ku;
        double KI = KP / (Tu / 2.0) * 0.002;
        double KD = KP * Tu / 2.0 / 0.002;

        for (int i = 0; i < st_params::valve::map.size(); i++) {
            pid.push_back(MiniPID(KP, KI, KD));
            pid[i].setOutputLimits(20);
            // setting a good output limit is important so as not to oscillate
        }
    }
    run = true;
    controller_thread = std::thread(&ValveController::controllerThread, this);
}

void ValveController::setSinglePressure(int index, int pressure) {
    if (0 <= index && index < st_params::valve::map.size())
        desired_pressures[index] = pressure;
    else
        std::cout << "error: actuator ID out of bounds: \t" << index << std::endl;
}

void ValveController::controllerThread() {
    // sensor_pressures and output_pressures use valve IDs for easier interfacing with mpa library
    std::vector<int> sensor_pressures(st_params::valve::num_valves);
    std::vector<int> output_pressures(st_params::valve::num_valves);
    logBeginTime = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < st_params::valve::num_valves; i++) {
        output_pressures[i] = 0;
    }
    int i = 0;
    int valve_id;
    while (run) {
        i++;
        mpa->get_all_pressures(&sensor_pressures);
        for (int i = 0; i < st_params::valve::map.size(); i++) {
            valve_id = st_params::valve::map[i];
            if (st_params::valve::use_pid)
                output_pressures[valve_id] = desired_pressures[i] + pid[i].getOutput(sensor_pressures[valve_id], desired_pressures[i]);
            else
                output_pressures[valve_id] = desired_pressures[i];
            // constrain pressure value to between 0 and max_pressure
            // valve goes haywire when it tries to write negative value
            output_pressures[valve_id] = std::max(0, std::min(output_pressures[valve_id], st_params::valve::max_pressure));
        }
        mpa->set_all_pressures(output_pressures);
        if (st_params::valve::log) {
            seconds_log.push_back((double) (std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::high_resolution_clock::now() - logBeginTime).count()) / 1000);
            sensor_pressure_log.push_back(sensor_pressures);
            desired_pressure_log.push_back(desired_pressures);
        }
    }

    // turn off all valves
    for (int i = 0; i < st_params::valve::num_valves; i++) {
        output_pressures[i] = 0;
    }
    mpa->set_all_pressures(output_pressures);
    mpa->disconnect();
}

void ValveController::write_log(){
    std::cout << "Outputting log of ValveController to log_pressure.csv..." << std::endl;
    std::ofstream log_file;
    log_file.open("log_pressure.csv");

    // write first line
    log_file << "time(sec)";
    for (int i = 0; i < st_params::valve::map.size(); ++i) {
        log_file <<", p_des[" << i<< "]";
    }
    for (int i = 0; i < st_params::valve::map.size(); ++i) {
        log_file <<", p_meas[" << i<< "]";
    }
    log_file<<"\n";

    // write data
    for (int j = 0; j < seconds_log.size(); ++j) {
        log_file << seconds_log[j];
        // use the actuator IDs, not valve IDs when writing out
        for (int i = 0; i < st_params::valve::map.size(); ++i) {
            log_file << ", " << desired_pressure_log[j][i];
        }
        for (int i = 0; i < st_params::valve::map.size(); ++i) {
            log_file << ", " << sensor_pressure_log[j][st_params::valve::map[i]];
        }
        log_file<<"\n";
    }
    log_file.close();
    std::cout<<"log_pressure.csv is output.\n";
}

void ValveController::disconnect() {
    run = false;
    controller_thread.join();
    std::cout << "Stopped ValveController thread." << std::endl;
    if (st_params::valve::log)
        write_log();
}

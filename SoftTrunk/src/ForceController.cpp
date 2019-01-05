// Copyright 2018 ...
#include "ForceController.h"


#define USE_PID true
#define LOG true

ForceController::ForceController(int maxValveIndex, int maxPressure) : DoF(maxValveIndex), maxPressure(maxPressure) {
    run = true;
    std::cout << "Connecting to MPA." << '\n';
    if (!mpa.connect()) {
        std::cout << "Failed to connect to MPA." << '\n';
        return;
    }
    std::cout << "Successfully connected to MPA." << '\n';
    // Ziegler-Nichols method
    double Ku = 2.6;
    double Tu = 0.14;
    double KP = 0.6 * Ku;
    double KI = KP / (Tu / 2.0) * 0.002;
    double KD = KP * Tu / 2.0 / 0.002;

    for (int i = 0; i < 16; i++) {
        commanded_pressures.push_back(0);
        pid.push_back(MiniPID(KP, KI, KD));
        pid[i].setOutputLimits(20);
        // setting a good output limit is important so as not to oscillate
    }
    controller_thread = std::thread(&ForceController::controllerThread, this);
}

void ForceController::setSinglePressure(int index, int pressure) {
    if (index < 0 || index >= DoF) {
        // wrong index
        return;
    }
    commanded_pressures[index] = pressure;
}

void ForceController::controllerThread() {
    std::vector<int> sensor_pressures(16);
    std::vector<int> output_pressures(16);
    logBeginTime = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 16; i++) {
        output_pressures[i] = 0;
    }
    int i = 0;
    while (run) {
        i++;
        mpa.get_all_pressures(&sensor_pressures);
        for (int i = 0; i < DoF; i++) {
            output_pressures[i] = commanded_pressures[i] + pid[i].getOutput(sensor_pressures[i], commanded_pressures[i]);
            if (output_pressures[i] < 0) {
                // valve goes haywire when it tries to write negative value
                output_pressures[i] = 0;
            }
        }
        for (int j = 0; j < DoF; ++j) {
            if (output_pressures[j] > maxPressure) {
                output_pressures[j] = maxPressure;
            }
        }
        if (USE_PID) {
            mpa.set_all_pressures(output_pressures);
        } else {
            mpa.set_all_pressures(commanded_pressures);
        }
        if (LOG) {
            seconds_log.push_back((double) (std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::high_resolution_clock::now() - logBeginTime).count()) / 1000);
            pressure_log.push_back(sensor_pressures);
            commandpressure_log.push_back(commanded_pressures);
        }
    }

    // turn off all valves
    for (int i = 0; i < 16; i++) {
        output_pressures[i] = 0;
    }
    mpa.set_all_pressures(output_pressures);
    mpa.disconnect();
}

void ForceController::disconnect() {
    run = false;
    controller_thread.join();
    std::cout << "Stopped ForceController thread.\n";
    std::cout << "Outputting log of ForceController to log_pressure.csv...\n";
    std::ofstream log_file;
    log_file.open("log_pressure.csv");
    log_file << "time(millis)";
    for (int i = 0; i < 16; ++i) {
        log_file <<", p_ref[" << i<< "]";
    }
    for (int i = 0; i < 16; ++i) {
        log_file <<", p_meas[" << i<< "]";
    }
    log_file<<"\n";
    for (int j = 0; j < seconds_log.size(); ++j) {
        log_file << seconds_log[j];
        for (int i = 0; i < 16; ++i) {
            log_file << ", "<<commandpressure_log[j][i];
        }
        for (int i = 0; i < 16; ++i) {
            log_file << ", "<<pressure_log[j][i];
        }
        log_file<<"\n";
    }
    log_file.close();
    std::cout<<"log_pressure.csv is output.\n";
}

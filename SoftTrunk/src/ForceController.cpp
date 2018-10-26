// Copyright 2018 ...
#include "ForceController.h"
#include <iostream>
#include <thread>

#define USE_PID true
#define PLOT true
/*
   This is a class that acts as a PID controller for the Festo valves.
   It sends out target pressures to the Festo valves.
   Runs as a separate thread from the main code, since it continuously does PID
   control.
 */
ForceController::ForceController(int DoF, int maxPresure) : DoF(DoF), maxPressure(maxPresure) {
  run = true;
  if (!mpa.connect()) {
    std::cout << "Failed to connect to MPA." << '\n';
    return;
  }
  // Ziegler-Nichols method
  double Ku = 2.6;
  double Tu = 0.14;
  double KP = 1.0; // 0.6 * Ku;
  double KI = 0.0; // KP/ (Tu / 2.0) * 0.002;
  double KD = 0.0; // KP*Tu/2.0 / 0.002;

  for (int i = 0; i < 16; i++) {
    commanded_pressures.push_back(0);
    pid.push_back(MiniPID(KP, KI, KD));
    pid[i].setOutputLimits(20);
    // setting a good output limit is important so as not oscillate
  }
  controller_thread = std::thread(&ForceController::controllerThread, this);
}

void ForceController::setSinglePressure(int index, int pressure) {
  // set pressure for a single valve.
  if (index < 0 || index >= DoF) {
    // wrong index
    return;
  }
  commanded_pressures[index] = pressure;
}

void ForceController::controllerThread() {
  std::vector<int> sensor_pressures(16);
  std::vector<int> output_pressures(16);
  for (int i = 0; i < 16; i++) {
    output_pressures[i] = 0;
  }
  int i = 0;
  int plot_cycles = 500;
  while (run) {
    i++;
    mpa.get_all_pressures(&sensor_pressures);
    for (int i = 0; i < DoF; i++) {
      output_pressures[i] =
          commanded_pressures[i] +
          pid[i].getOutput(sensor_pressures[i], commanded_pressures[i]);
      if (output_pressures[i] < 0) {
        // goes haywire when it tries to write negative value
        output_pressures[i] = 0;
      }
    }
    for (int j = 0; j < DoF; ++j) {
      if (output_pressures[j]>maxPressure){
        output_pressures[j] = maxPressure;
      }
    }
    if (USE_PID) {
      mpa.set_all_pressures(output_pressures);
    } else {
      mpa.set_all_pressures(commanded_pressures);
    }
    if (PLOT) {
      x.push_back(i);
      pressure_log.push_back(sensor_pressures[0]);
      commandpressure_log.push_back(commanded_pressures[0]);
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
  if (PLOT) {
    namespace plt = matplotlibcpp;
    plt::figure_size(1200, 780);
    plt::named_plot("measured pressure", x, pressure_log);
    plt::named_plot("commanded pressure", x, commandpressure_log);
    plt::legend();
    plt::save("./graph.png");
    std::cout << "graph output to ./graph.png" << '\n';
  }
}

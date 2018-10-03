// Copyright 2018 ...
#include "./forceController.h"
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
ForceController::ForceController(int DoF) : DoF(DoF) {
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
  return;
}

void ForceController::controllerThread() {
  std::vector<int> sensor_pressures(16);
  std::vector<int> output_pressures(16);
  for (int i = 0; i < 16; i++) {
    output_pressures[i] = 0;
  }
  int i = 0;
  int plot_cycles = 500;
  std::vector<double> x(plot_cycles), pressure_log(plot_cycles),
      commandpressure_log(plot_cycles);

  while (run) {
    std::cout << output_pressures[0] << '\n';
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
    if (USE_PID) {
      mpa.set_all_pressures(output_pressures);
    } else {
      mpa.set_all_pressures(commanded_pressures);
    }
    if (PLOT) {
      i++;
      if (i < plot_cycles) {
        x.at(i) = i;
        pressure_log.at(i) = sensor_pressures[0];
        commandpressure_log.at(i) = commanded_pressures[0];
      }
    }
  }

  // turn off all valves
  for (int i = 0; i < 16; i++) {
    output_pressures[i] = 0;
  }
  mpa.set_all_pressures(output_pressures);
  mpa.disconnect();
  if (PLOT) {
    namespace plt = matplotlibcpp;
    plt::figure_size(1200, 780);
    plt::named_plot("measured pressure", x, pressure_log);
    plt::named_plot("commanded pressure", x, commandpressure_log);
    // plt::ylim(800,1200);
    plt::legend();
    plt::save("./graph.png");
    std::cout << "graph output to ./graph.png" << '\n';
  }
}
void ForceController::disconnect() {
  run = false;
  controller_thread.join();
}

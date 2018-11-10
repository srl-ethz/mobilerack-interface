// Copyright 2018 ...
#include "ForceController.h"


#define USE_PID true
#define PLOT true
#define VALVE_TO_PLOT 14
/*
   This is a class that acts as a PID controller for the Festo valves.
   It sends out target pressures to the Festo valves.
   Runs as a separate thread from the main code, since it continuously does PID
   control.
 */
ForceController::ForceController(int DoF, int maxPresure) : DoF(DoF), maxPressure(maxPresure) {
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
  double KI = KP/ (Tu / 2.0) * 0.002;
  double KD = KP*Tu/2.0 / 0.002;

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
      seconds_log.push_back((double)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - logBeginTime).count())/1000);
      pressure_log.push_back(sensor_pressures[VALVE_TO_PLOT]);
      commandpressure_log.push_back(commanded_pressures[VALVE_TO_PLOT]);
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
    plt::named_plot("measured pressure", seconds_log, pressure_log);
    plt::named_plot("commanded pressure", seconds_log, commandpressure_log);
    plt::legend();
    plt::save("./graph.png");
    std::cout << "graph output to ./graph.png" << '\n';
  }
}

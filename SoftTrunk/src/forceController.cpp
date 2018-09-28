// Copyright 2018 ...
#include "./forceController.h"
#include <iostream>
#include "../include/mpa/mpa.h"
#include "./MiniPID/MiniPID.h"
#include "./matplotlib-cpp/matplotlibcpp.h"

#define USE_PID true

/*
This sends out target pressures to the Festo valves.
Runs as a separate thread from the main code, since it continuously does PID control.
*/
ForceController::ForceController(int DoF):DoF(DoF) {
  run = true;
  MPA mpa("192.168.1.101", "502");
  if (!mpa.connect()) {
    std::cout << "Failed to connect to MPA." << '\n';
    return;
  }
  for (int i = 0; i < 16; i++) {
    commanded_pressures.push_back(0);
  }
}

void ForceController::controllerThread() {
  std::vector<int> sensor_pressures(16);
  std::vector<int> output_pressures(16);
  for (int i=0; i < 16; i++) {
    output_pressures[i] = 0;
  }
  while (run) {
    mpa.get_all_pressures(sensor_pressure);
    for (int i=0; i < DoF; i++) {
      output_pressures[i] = pid[i].getOutput(sensor_pressures[i], command_pressures[i]);
    }
    if (USE_PID) {
      mpa.set_all_pressures(output_pressures);
    } else {
      mpa.set_all_pressures(command_pressures);
    }
  }

  // turn off all valves
  for(int i=0; i<16; i++){
    output_pressures[i] = 0;
  }
  mpa.set_all_pressures(output_pressures);
  mpa.disconnect();
}

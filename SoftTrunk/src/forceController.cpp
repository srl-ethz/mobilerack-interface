// Copyright 2018 ...
#include "./forceController.h"
#include <iostream>
#include <thread>

#define USE_PID true

/*
This sends out target pressures to the Festo valves.
Runs as a separate thread from the main code, since it continuously does PID control.
*/
ForceController::ForceController(int DoF):DoF(DoF) {
  run = true;
  if (!mpa.connect()) {
    std::cout << "Failed to connect to MPA." << '\n';
    return;
  }
  // Ziegler-Nichols method
  double Ku = 2.6;
  double Tu = 0.14;
  double KP = 0.6 * Ku;
  double KI = KP/ (Tu / 2.0) * 0.002;
  double KD = KP*Tu/2.0 / 0.002;

  for (int i = 0; i < 16; i++) {
    commanded_pressures.push_back(0);
    pid.push_back(MiniPID(KP, KI, KD));
  }
  std::thread controller_thread(&ForceController::controllerThread, this);
}

void ForceController::setSinglePressure(int index, int pressure) {
  // set pressure for a single valve.
  if  (index<0 || index>=DoF) {
    // wrong index
    return;
  }
  commanded_pressures[index] = pressure;
  return;
}

void ForceController::controllerThread() {
  std::vector<int> sensor_pressures(16);
  std::vector<int> output_pressures(16);
  for (int i=0; i < 16; i++) {
    output_pressures[i] = 0;
  }
  while (run) {
    mpa.get_all_pressures(&sensor_pressures);
    for (int i=0; i < DoF; i++) {
      output_pressures[i] = commanded_pressures[i] + pid[i].getOutput(sensor_pressures[i], commanded_pressures[i]);
    }
    if (USE_PID) {
      mpa.set_all_pressures(output_pressures);
    } else {
      mpa.set_all_pressures(commanded_pressures);
    }
  }

  // turn off all valves
  for(int i=0; i<16; i++){
    output_pressures[i] = 0;
  }
  mpa.set_all_pressures(output_pressures);
  mpa.disconnect();
}
void ForceController::disconnect(){
  run = false;
}
/*
int main(){
  return 1;
}
*/

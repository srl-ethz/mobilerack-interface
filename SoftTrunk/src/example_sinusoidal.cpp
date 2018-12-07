// Copyright 2018 ...
#include "ForceController.h"
#include <cmath>
#include <iostream>
#include <thread>
/*
sends a sinusoidal wave to 3 actuators, and each phase is offset by 2*PI/3.
*/
#define DURATION 10 //in seconds
#define WAIT 0.01 // in seconds
#define PI 3.1415
void wait() { std::this_thread::sleep_for(std::chrono::milliseconds((int)(WAIT*1000))); }
std::vector<int> valve_map={0,1,2};

int sinusoid(double t) { return 400 + 100 * sin(t); }
int main() {
  ForceController forceController(16, 1000);

  for (double time = 0; time < DURATION; time+=WAIT) {
    forceController.setSinglePressure(valve_map[0], sinusoid(time));
    forceController.setSinglePressure(valve_map[1], sinusoid(time+2*PI/3));
    forceController.setSinglePressure(valve_map[2], sinusoid(time+4*PI/3));
    wait();
  }
  forceController.disconnect();
  return 1;
}

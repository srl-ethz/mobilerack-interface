// Copyright 2018 ...
#include "forceController.h"
#include <cmath>
#include <iostream>
#include <thread>
#define PI 3.141592
/*
sends a sinusoidal wave to 4 actuators, and each phase is offset by PI/2.
*/
void wait() { std::this_thread::sleep_for(std::chrono::milliseconds(10)); }

int sinusoid(double t) { return 500 + 400 * sin(t); }
int main() {
  ForceController forceController(4);

  for (int i = 0; i < 10000; i++) {
    for (int j = 0; j < 4; j++) {
      forceController.setSinglePressure(
          j, sinusoid(static_cast<double>(i) / 40 + PI * j / 2));
    }
    wait();
  }
  forceController.disconnect();
  return 1;
}

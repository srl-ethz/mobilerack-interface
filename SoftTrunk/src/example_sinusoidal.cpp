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

int sinusoid(double t) { return 200 + 200 * sin(t); }
int main() {
  ForceController forceController(16);

  for (int i = 0; i < 10000; i++) {
    for (int j = 0; j < 4; j++) {
      forceController.setSinglePressure(
          j+4, sinusoid(static_cast<double>(i) / 20 + PI * j / 2));
      forceController.setSinglePressure(
              j+8, sinusoid(static_cast<double>(i) / 20 + PI * j / 2 + PI * 2/3));
      forceController.setSinglePressure(
              j+12, sinusoid(static_cast<double>(i) / 20 + PI * j / 2 + PI * 4/3));
    }
    wait();
  }
  forceController.disconnect();
  return 1;
}

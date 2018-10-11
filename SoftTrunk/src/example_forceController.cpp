#include "forceController.h"
#include <chrono>
#include <iostream>
#include <thread>
/*
This program shows an example of using the forceController.
forceController is basically an interface to the Festo valves that also
implements a PID loop for each valve.
*/
void wait() { std::this_thread::sleep_for(std::chrono::milliseconds(500)); }
int main() {
  ForceController forceController(4);
  /*
    for (int i = 0; i < 20; i++) {
      forceController.setSinglePressure(i % 4, 1200);
      forceController.setSinglePressure((i + 2) % 4, 200);
      wait();
      forceController.setSinglePressure(i % 4, 0);
      forceController.setSinglePressure((i + 2) % 4, 0);
    }
    */
  forceController.setSinglePressure(2, 1500);
  wait();
  forceController.disconnect();

  return 1;
}

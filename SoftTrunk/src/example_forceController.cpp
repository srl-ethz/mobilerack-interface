#include "forceController.h"
#include <chrono>
#include <iostream>
#include <thread>
/*
This program shows an example of using the forceController.
forceController is basically an interface to the Festo valves that also
implements a PID loop for each valve.
*/
void wait() { std::this_thread::sleep_for(std::chrono::milliseconds(1000)); }
int main() {
  ForceController forceController(16);
  for (int i = 0; i < 100; i++) {
    forceController.setSinglePressure(i % 12+4, 400);
    wait();
    forceController.setSinglePressure((i) % 12+4, 0);
    //forceController.setSinglePressure(i % 12, 0);
    //forceController.setSinglePressure((i + 2) % 12, 0);
  }
  wait();
  forceController.disconnect();

  return 1;
}

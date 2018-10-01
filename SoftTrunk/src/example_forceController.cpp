#include "./forceController.h"
#include <iostream>
#include <chrono>
#include <thread>

void wait() {
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
int main(){
  ForceController forceController(1);

  forceController.setSinglePressure(0,800);

  return 1;
}

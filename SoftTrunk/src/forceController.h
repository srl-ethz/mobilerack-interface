#ifndef FORCECONTROLLER_H
#define FORCECONTROLLER_H

#include <vector>
#include <thread>
#include "./MiniPID/MiniPID.h"
#include "../include/mpa/mpa.h"
#include "./matplotlib-cpp/matplotlibcpp.h"


class ForceController{
 private:
  void controllerThread();
  bool run;
  std::vector<int> commanded_pressures;
  int DoF;
  std::vector<MiniPID> pid;
  MPA mpa{"192.168.1.101", "502"};
  std::thread controller_thread;
 public:
  void setSinglePressure(int, int);
  explicit ForceController(int);
  void disconnect();
};
#endif

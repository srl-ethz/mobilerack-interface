#ifndef FORCECONTROLLER_H
#define FORCECONTROLLER_H

#include "MiniPID.h"
#include "matplotlibcpp.h"
#include "mpa.h"
#include <thread>
#include <vector>

class ForceController {
private:
  void controllerThread();
  bool run;
  std::vector<int> commanded_pressures;
  int DoF;
  std::vector<MiniPID> pid;
  MPA mpa{"192.168.1.101", "502"};
  std::thread controller_thread;
  std::vector<double> x;
  std::vector<double> pressure_log;
  std::vector<double> commandpressure_log;

public:
  void setSinglePressure(int, int);
  explicit ForceController(int);
  void disconnect();
};
#endif

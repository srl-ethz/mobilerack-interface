#ifndef FORCECONTROLLER_H
#define FORCECONTROLLER_H

#include <vector>
#include "./MiniPID/MiniPID.h"
#include "../include/mpa/mpa.h"


class ForceController{
 private:
  void controllerThread();
  bool run;
  std::vector<int> commanded_pressures;
  int DoF;
  std::vector<MiniPID> pid;
  MPA mpa{"192.168.1.101", "502"};
 public:
  void setSinglePressure(int index, int pressure);
  explicit ForceController(int DoF);
  void disconnect();
};
#endif

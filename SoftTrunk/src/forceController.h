#ifndef FORCECONTROLLER_H
#define FORCECONTROLLER_H

class ForceController{
private:
  void controllerThread(int DoF);
  bool run;
  std::vector<int> commanded_pressure(16);
  int DoF;

public:

};

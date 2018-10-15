// Copyright 2018 YT
#ifndef SOFTTRUNK_INCLUDE_CURVATURESENSOR_H_
#define SOFTTRUNK_INCLUDE_CURVATURESENSOR_H_

#include "OptiTrackClient.h"
class CurvatureSensor {
  /*
  handles the calculation of curvature. Gets the current pose of the robot, then
  converts it to a list of curvatures (theta and phi).
  */
private:
  void getNewData();
  void extractPositionData(); // convert current position data from OptiTrack to
                              // theta and phi values
  int _numOfRigidBodies;
  uint32_t _localAddress;
  uint32_t _serverAddress;

public:
  explicit CurvatureSensor(int numOfRigidBodies, uint32_t localAddress,
                           uint32_t serverAddress);
  void start();
  void stop();
  std::vector<double> theta; // offset angle from x axis
  std::vector<double> phi;   // degree of curvature
};

#endif // SOFTTRUNK_INCLUDE_CURVATURESENSOR_H_

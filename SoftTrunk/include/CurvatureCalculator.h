// Copyright 2018 Yasu
#ifndef SOFTTRUNK_INCLUDE_CurvatureCalculator_H_
#define SOFTTRUNK_INCLUDE_CurvatureCalculator_H_

#define USE_OPTITRACK 0
#define USE_INTEGRATEDSENSOR 1

#include "OptiTrackClient.h"

#include <Eigen/Geometry>

class CurvatureCalculator {
  /*
  handles the calculation of curvature. Gets the current pose of the robot, then
  converts it to a list of curvatures (theta and phi).
  */
private:
  void extractPositionData(); // convert current position data from OptiTrack to
                              // theta and phi values
  int numOfRigidBodies;
  int sensorType;
  OptiTrackClient *optiTrackClient;
  std::vector<Eigen::Translation<double, 3>> positions; // save positions
  std::vector<Eigen::Quaternion<double>> quaternions;   // save quaternions
  // std::vector<Eigen::Transform<double, 3, Eigen::Affine>> transforms;
  std::vector<Eigen::Matrix4d> transforms;

public:
  explicit CurvatureCalculator(int numOfRigidBodies, int sensorType);
  /*
  sensorType: USE_OPTITRACK or USE_INTEGRATEDSENSOR
  */

  void setupOptiTrack(std::string localAddress, std::string serverAddress);
  /*
  uses the optitrack system to measure curvature.
  id conventions:
  base is 0, first frame after that is 1, and so on...
  */
  void setupIntegratedSensor(); // for future, if you want to use sensors
                                // embedded in arm.
  void stop();
  void calculateCurvature(); // calculate current theta and phi values
  std::vector<double> theta; // offset angle from x axis
  std::vector<double> phi;   // degree of curvature
};

#endif // SOFTTRUNK_INCLUDE_CurvatureCalculator_H_

#ifndef SOFTTRUNK_INCLUDE_CurvatureCalculator_H_
#define SOFTTRUNK_INCLUDE_CurvatureCalculator_H_

#define USE_OPTITRACK 0
#define USE_INTEGRATEDSENSOR 1

#include "OptiTrackClient.h"
#include "SoftTrunk_common_defs.h"

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <thread>
#include <cmath>

class CurvatureCalculator {
  /*
  handles the calculation of curvature. Gets the current pose of the robot, then
  converts it to a list of curvatures (theta and phi).
  */
private:
  int numOfRigidBodies;
  int sensorType;
  OptiTrackClient *optiTrackClient;
  std::vector<Eigen::Transform<double, 3, Eigen::Affine>> abs_transforms;
  std::vector<Eigen::Transform<double, 3, Eigen::Affine>> rel_transforms;
  std::thread calculatorThread;

  void calculatorThreadFunction(); // backgrond process for calculating curvature
  bool run;
  void calculateCurvature(); // calculates phi and theta from the current frame values.

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
  void setupIntegratedSensor(); // for future, if you want to use sensors embedded in arm.
  void start(); //calling this starts a thread that continuously calculates theta, phi, and their time derivatives.
  void stop(); // stops the thread, disconnects.

  Vector2Nd q = Vector2Nd::Zero(); // [phi0, theta0, phi1, theta1, ...]
  Vector2Nd dq = Vector2Nd::Zero(); // derivative of dq
  Vector2Nd ddq = Vector2Nd::Zero(); // derivative of dq
};

#endif // SOFTTRUNK_INCLUDE_CurvatureCalculator_H_

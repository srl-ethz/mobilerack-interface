#ifndef ARMPCC_H
#define ARMPCC_H

#define NUM_ELEMENTS 3

#include <rbdl/rbdl.h>
#ifndef RBDL_BUILD_ADDON_URDFREADER
	#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>

#include <iostream>
#include <fstream>
#include "forceController.h"
#include <Eigen>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class ArmElement{
  // one PCC element in the arm.
private:
  double length;
  Model rbdl_model;
public:
  ArmElement(double length);
};

class Arm{
  // the PCC Arm itself.
private:
  ArmElement* armElements[NUM_ELEMENTS];
  std::vector<double> lengths;
  std::vector<double> masses;
  Model* rbdl_model;
  void create_rbdl_model();
  void create_actual_model();
  ForceController* forceController;
  std::vector<double>
public:
  Arm(bool create_urdf=false);
  void create_urdf(); //generate a file robot.urdf.xacro, using the lengths and masses of the actual robot.
  Eigen::Matrix<double, NUM_ELEMENTS*8, 1> xi; // map from config to augmented
  Eigen::Matrix<double, NUM_ELEMENTS*8, NUM_ELEMENTS*2> Jm;
  Eigen::Matrix<double, NUM_ELEMENTS*8, NUM_ELEMENTS*2> dJm;
  void update(Eigen::Matrix<double, NUM_ELEMENTS*2, 1>, Eigen::Matrix<double, NUM_ELEMENTS*2, 1>); // update the member variables based on current values
  void actuate();
  void setTargetForces();
};
#endif

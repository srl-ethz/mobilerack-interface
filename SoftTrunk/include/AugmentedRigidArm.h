#ifndef ARMPCC_H
#define ARMPCC_H

#define NUM_ELEMENTS 3  // how many PCC segments there are
#define USE_ROS false  // do you want to publish joint states to ROS

#include <rbdl/rbdl.h>
#ifndef RBDL_BUILD_ADDON_URDFREADER
	#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>

#include <iostream>
#include <fstream>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


class AugmentedRigidArm{
  /* class for Augmented Model of the PCC arm.
   * It is supposed to be a kinematic & dynamic analogue of the soft arm.
   * this class can calculate the kinematics & dynamics of the augmented arm.
   */
private:
  std::vector<double> lengths; // length of each segment (probably meters)
  std::vector<double> masses; // mass of each segment (probably kg)
  Model* rbdl_model; // RBDL model of arm
  void create_rbdl_model(); // create a RBDL model and save it to rbdl_model
  void joint_publish(); // publish joint state to ROS
  void extract_B_G();  // extract inertia matrix and gravity vector.
public:
  explicit AugmentedRigidArm(bool is_create_xacro=false); // set is_create_xacro to true if you want to generate the model's
  void create_xacro(); //generate a file robot.urdf.xacro, using the lengths and masses of the actual robot.
  Eigen::Matrix<double, NUM_ELEMENTS*8, 1> xi; // map from config to augmented space
  Eigen::Matrix<double, NUM_ELEMENTS*8, NUM_ELEMENTS*2> Jm; // Jacobian
  Eigen::Matrix<double, NUM_ELEMENTS*8, NUM_ELEMENTS*2> dJm; // time derivative of Jacobian
  Eigen::Matrix<double, NUM_ELEMENTS*8, NUM_ELEMENTS*8> B_xi; // inertia matrix
  Eigen::Matrix<double, NUM_ELEMENTS*8, 1> G_xi; //gravity vector
  void update(Eigen::Matrix<double, NUM_ELEMENTS*2, 1>, Eigen::Matrix<double, NUM_ELEMENTS*2, 1>); // update the member variables based on current values
};
#endif

#ifndef ARMPCC_H
#define ARMPCC_H

#define USE_ROS false  // do you want to publish joint states to ROS

#include <rbdl/rbdl.h>
#include "SoftTrunk_common_defs.h"

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

  Model* rbdl_model; // RBDL model of arm
  void create_rbdl_model(); // create a RBDL model and save it to rbdl_model
  void joint_publish(); // publish joint state to ROS
  void extract_B_G();  // extract inertia matrix and gravity vector.
  void create_xacro(); //generate a file robot.urdf.xacro, using the lengths and masses of the actual robot.
public:
  explicit AugmentedRigidArm(bool is_create_xacro=false); // set is_create_xacro to true if you only want to generate the model's xacro model
  std::vector<double> lengths; // length of each segment (probably meters)
  std::vector<double> masses; // mass of each segment (probably kg)
  Eigen::Matrix<double, NUM_ELEMENTS*6, 1> xi; // map from config to augmented space
  Eigen::Matrix<double, NUM_ELEMENTS*6, NUM_ELEMENTS*2> Jxi=Eigen::Matrix<double, NUM_ELEMENTS*6, NUM_ELEMENTS*2>::Zero(); // Jacobian
  Eigen::Matrix<double, NUM_ELEMENTS*6, NUM_ELEMENTS*2> dJxi=Eigen::Matrix<double, NUM_ELEMENTS*6, NUM_ELEMENTS*2>::Zero(); // time derivative of Jacobian
  Eigen::Matrix<double, 3, NUM_ELEMENTS*2> update_J(Vector2Nd q); // used for inverse kinematics

  Eigen::Matrix<double, NUM_ELEMENTS*6, NUM_ELEMENTS*6> B_xi; // inertia matrix
  Eigen::Matrix<double, NUM_ELEMENTS*6, 1> G_xi; //gravity vector
  void update(Vector2Nd, Vector2Nd); // update the member variables based on current values
  void update_xi(Vector2Nd);
  void update_Jxi(Vector2Nd);
  void update_dJxi(Vector2Nd, Vector2Nd);
};
#endif

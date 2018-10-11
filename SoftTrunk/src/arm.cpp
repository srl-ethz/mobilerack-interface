// Copyright 2018 ...
#include "./arm.h"
#include <iostream>
#include <rbdl/rbdl.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
  #error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>
using RigidBodyDynamics;
using RigidBodyDynamics::Math;

using std::cout;

Arm::Arm() {
  create_rbdl_model();
  create_actual_model();
}

Arm::create_rbdl_model() {
    Model* rbdl_model = new Model();
  if (!Addons::URDFReadFromFile("../urdf/robot.urdf", rbdl_model, false)) {
    std::cerr << "Error loading model ../urdf/robot.urdf" << std::endl;
    abort();
  }
  rbdl_model->gravity = Vector3d(0., 0., -9.81);
  // DoF is model->dof_count. If I can do ARM_ELEMENTS=rbdl_model->dof_count/6, it would be a much better solution.
  // but since I still am not sure how to make variable length arrays in C++, will just define it separately as a constant.
}

Arm::create_actual_model() {
  for (int i=0; i < ARM_ELEMENTS; i++) {
    double length = 0.2;
    armElements[i]= new ArmElement(length);
  }
}
Arm::setTargetForces() {

}
Arm::actuate()

ArmElement::ArmElement(double length):length(length) {
}

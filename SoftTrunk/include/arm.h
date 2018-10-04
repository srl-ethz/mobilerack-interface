#ifndef ARMPCC_H
#define ARMPCC_H

#define ARM_ELEMENTS 3

#ifndef RBDL_BUILD_ADDON_URDFREADER
	#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>
using RigidBodyDynamics;
using RigidBodyDynamics::Math;


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
  ArmElement* armElements[ARM_ELEMENTS];
public:
  Arm();
};
#endif

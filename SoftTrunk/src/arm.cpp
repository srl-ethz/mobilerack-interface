// Copyright 2018 ...
#include "arm.h"



Arm::Arm(bool create_urdf) {
    for (int i = 0; i < NUM_ELEMENTS; ++i) {
        lengths.push_back(0.3);
        masses.push_back(1000.0);
    }
    if (create_urdf){
        Arm::create_urdf();
        return;
    }
  Arm::create_rbdl_model();
  Arm::create_actual_model();
}

void Arm::create_urdf(){
    std::cout << "generating URDF XACRO file robot.urdf.xacro...";
    std::ofstream xacro_file;
    xacro_file.open("./urdf/robot.urdf.xacro");

    // write out text to the file
    xacro_file << "<?xml version='1.0'?><robot xmlns:xacro='http://www.ros.org/wiki/xacro' name='robot'>"<<
    "<xacro:include filename='macro_definitions.urdf.xacro' /><xacro:empty_link name='base_link'/>";

    // write out first PCC element
    // this is written outside for loop because parent of first PCC element must be called base_link
    xacro_file << "<xacro:PCC id='0' parent='base_link' child='mid-0' length='"<< lengths[0] << "' mass='" << masses[0] <<"'/>"<<
    "<xacro:empty_link name='mid-0'/>";
    // iterate over all the other PCC elements
    for (int i = 1; i < NUM_ELEMENTS; ++i) {
        xacro_file << "<xacro:PCC id='"<< i <<"' parent='"<< "mid-" << i-1 << "' child='"<< "mid-" << i <<"' length='"<< lengths[i] <<"' mass='"<< masses[i] <<"'/>"<<
        "<xacro:empty_link name='"<< "mid-" << i <<"'/>";
    }
    xacro_file << "</robot>";

    xacro_file.close();
}

void Arm::create_rbdl_model() {
    auto rbdl_model = new RigidBodyDynamics::Model();
  if (!RigidBodyDynamics::Addons::URDFReadFromFile("../urdf/robot.urdf", rbdl_model, false)) {
    std::cerr << "Error loading model ../urdf/robot.urdf" << std::endl;
    abort();
  }
  rbdl_model->gravity = Vector3d(0., 0., -9.81);

}

void Arm::create_actual_model() {
  for (int i=0; i < NUM_ELEMENTS; i++) {
    double length = 0.2;
    armElements[i]= new ArmElement(length);
  }
}
void Arm::setTargetForces() {

}
void Arm::actuate(){

}

ArmElement::ArmElement(double length):length(length) {
}

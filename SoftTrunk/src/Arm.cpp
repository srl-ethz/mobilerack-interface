// Copyright 2018 ...
#include "Arm.h"


Arm::Arm(bool create_urdf) {
    for (int i = 0; i < NUM_ELEMENTS; ++i) {
        lengths.push_back(0.2);
        masses.push_back(1000.0);
    }
    if (create_urdf){
        Arm::create_urdf();
        return;
    }
    forceController = new ForceController{NUM_ELEMENTS*4, 400};
  create_rbdl_model();
  create_actual_model();
  if (USE_ROS){
//      https://stackoverflow.com/questions/50324348/can-a-ros-node-be-created-outside-a-catkin-workspace
//      ros::init("", "", "joint_pub");
//      ros::NodeHandle n;
  }
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
  if (!RigidBodyDynamics::Addons::URDFReadFromFile("./urdf/robot.urdf", rbdl_model, false)) {
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

void Arm::update(Eigen::Matrix<double, NUM_ELEMENTS*2,1> q, Eigen::Matrix<double, NUM_ELEMENTS*2, 1> dq) {
    double phi;
    double theta;
    double delta_L;
    double L_c;
    double dL_c;
    // first update xi (augmented model parameters)
    for (int i = 0; i < NUM_ELEMENTS; ++i) {
        phi = q(2*i+0);
        theta = q(2*i+1);
        delta_L = lengths[i]*sin(theta/2)/(theta/2);
        xi(8*i+0,0) = phi;
        xi(8*i+1,0) = theta/2;
        xi(8*i+2,0) = delta_L/2;
        xi(8*i+3,0) = -phi;
        xi(8*i+4,0) = phi;
        xi(8*i+5,0) = delta_L/2;
        xi(8*i+6,0) = theta/2;
        xi(8*i+7,0) = -phi;

        // next, update the Jacobian Jm
        L_c = lengths[i] * (theta*cos(theta/2)-2*sin(theta/2)) / (2*theta*theta);
        // first, make the values in Jm and dJm are all zeros
        for (int j = 0; j < NUM_ELEMENTS * 2; ++j) {
            for (int k = 0; k < 8; ++k) {
                Jm(8*i+k, j) = 0.0;
                dJm(8*i+k, j) = 0.0;
            }
        }
        Jm(8*i+0,2*i+0) = 1;
        Jm(8*i+1,2*i+1) = 0.5;
        Jm(8*i+2,2*i+1) = L_c;
        Jm(8*i+3,2*i+0) = -1;
        Jm(8*i+4,2*i+0) = 1;
        Jm(8*i+5,2*i+1) = L_c;
        Jm(8*i+6,2*i+1) = 0.5;
        Jm(8*i+7,2*i+0) = -1;

        // next, update dJm.
        dL_c = lengths[i] * dq(2*i+1) * (4*sin(theta/2)/pow(theta,3) - 2*cos(theta/2)/pow(theta, 2) - sin(theta/2)/(2*theta));
        dJm(8*i+2, 2*i+1) = dL_c;
        dJm(8*i+5, 2*i+1) = dL_c;
    }

    extract_B_G();
}

void Arm::extract_B_G() {
    // the fun part- extracting the B_xi(inertia matrix) and G_xi(gravity) from RBDL
    
    // first run ID with dQ and ddQ as zero vectors (gives gravity vector)
    VectorNd dQ_zeros = VectorNd::Zero(NUM_ELEMENTS*8);
    VectorNd ddQ_zeros = VectorNd::Zero(NUM_ELEMENTS*8);
    VectorNd tau = VectorNd::Zero(NUM_ELEMENTS*8);
    InverseDynamics(*rbdl_model, xi, dQ_zeros, ddQ_zeros, tau);
    G_xi = tau;
    
    // next, iterate through by making ddQ_zeros a unit vector and get inertia matrix
    for (int i = 0; i < NUM_ELEMENTS*8; ++i) {
        for (int j = 0; j < NUM_ELEMENTS * 8; ++j) {
            ddQ_zeros(j) = 0.0;
        }
        ddQ_zeros(i) = 1.0;
        InverseDynamics(*rbdl_model, xi, dQ_zeros, ddQ_zeros, tau);
        B_xi.col(i) = tau - G_xi;
    }
}

void Arm::joint_publish(){

}

ArmElement::ArmElement(double length):length(length) {
}
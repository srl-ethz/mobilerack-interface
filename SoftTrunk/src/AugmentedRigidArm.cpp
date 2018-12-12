// Copyright 2018 ...
#include "AugmentedRigidArm.h"


AugmentedRigidArm::AugmentedRigidArm(bool is_create_xacro) {
    rbdl_check_api_version (RBDL_API_VERSION);

    if (is_create_xacro){
        create_xacro();
        return;
    }
  create_rbdl_model();
  if (USE_ROS){
//      https://stackoverflow.com/questions/50324348/can-a-ros-node-be-created-outside-a-catkin-workspace
//      ros::init("", "", "joint_pub");
//      ros::NodeHandle n;
  }
}

void AugmentedRigidArm::create_xacro(){
    std::cout << "generating XACRO file robot.urdf.xacro...";
    std::ofstream xacro_file;
    //std::string pathToUse1 = "./urdf";
    //boost::filesystem::path dir1(pathToUse1);


  //if (!(boost::filesystem::exists(dir1))) {
  //    std::cout << "ERROR: following path doesn't exist" << std::endl;
  //    return;
  //}
    xacro_file.open("./urdf/robot.urdf.xacro");

    // write outlog text to the file
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
    std::cout << "Finished generation. Run ./create_urdf in /urdf directory to generate robot.urdf from robot.urdf.xacro.(requires ROS)\n";
}

void AugmentedRigidArm::create_rbdl_model() {
    rbdl_model = new RigidBodyDynamics::Model();
    if (!RigidBodyDynamics::Addons::URDFReadFromFile("./urdf/robot.urdf", rbdl_model, false)) {
        std::cerr << "Error loading model ./urdf/robot.urdf. Make sure that robot.urdf.xacro is generated with ./create_xacro, and then converted to robot.urdf with ./create_urdf.sh (requires ROS)" << std::endl;
        abort();
      }
      rbdl_model->gravity = Vector3d(0., 0., 9.81);
      std::cout << "Robot model created, with " << rbdl_model->dof_count << " DoF. \n";
}

void  AugmentedRigidArm::update_xi(Vector2Nd q) {
    double beta; double theta; double phi; double thetaX; double thetaY; double deltaL;
    if(CHAMBERS ==3)
        beta=2*PI/3;
    else if(CHAMBERS==4)
        beta=PI/2;
    for (int i = 0; i < NUM_ELEMENTS; ++i) {
        // use phi, theta parametrization because it's simpler for calculation
        if (q(2*i) == 0) {
            if (q(2 * i + 1) == 0)
                phi = 0;
            else
                phi = PI / 2;
        }
        else
            phi = atan((q(2*i+1)/q(2*i)-cos(beta))/sin(beta));

        if(phi ==PI/2)
            theta = -q(2*i+1)/(TRUNK_RADIUS*cos(beta-phi));
        else
            theta = -q(2*i)/(TRUNK_RADIUS*cos(phi));
        std::cout<<"phi" <<phi<<"\ntheta"<<theta<<"\n";

        thetaX = atan(tan(theta/2)*sin(phi));
        thetaY = asin(sin(theta/2)*cos(phi));
        if (theta==0)
            deltaL = 0;
        else
            deltaL = lengths[i]*sin(theta/2)/theta;

        xi(6*i+0) = thetaX;
        xi(6*i+1) = thetaY;
        xi(6*i+2) = deltaL;
        xi(6*i+3) = deltaL;
        xi(6*i+4) = thetaX;
        xi(6*i+5) = thetaY;
    }
}

void AugmentedRigidArm::update_Jxi(Vector2Nd q) {
    // brute force calculate Jacobian lol
    //todo: verify that this numerical method is actually okay
    // this particular method only works because xi of each element is totally independent of other elements
    Vector2Nd q_deltaX = Vector2Nd(q);
    Vector2Nd q_deltaY = Vector2Nd(q);
    double epsilon = 0.01;
    for (int i = 0; i < NUM_ELEMENTS; ++i) {
        q_deltaX(2*i) += epsilon;
        q_deltaY(2*i+1) += epsilon;
    }
    Eigen::Matrix<double, 6*NUM_ELEMENTS, 1> xi_current = Eigen::Matrix<double, 6*NUM_ELEMENTS, 1>(xi);
    update_xi(q_deltaX);
    Eigen::Matrix<double, 6*NUM_ELEMENTS, 1> xi_deltaX = Eigen::Matrix<double, 6*NUM_ELEMENTS, 1>(xi);
    update_xi(q_deltaY);
    Eigen::Matrix<double, 6*NUM_ELEMENTS, 1> xi_deltaY = Eigen::Matrix<double, 6*NUM_ELEMENTS, 1>(xi);
    for (int j = 0; j < NUM_ELEMENTS; ++j) {
        Jxi.block(6*j, 2*j+0, 6, 1) = (xi_deltaX-xi_current).block(6*j,0,6,1)/epsilon;
        Jxi.block(6*j, 2*j+1, 6, 1) = (xi_deltaY-xi_current).block(6*j,0,6,1)/epsilon;
    }
}

void AugmentedRigidArm::update_dJxi(Vector2Nd q, Vector2Nd dq) {
    //todo
    update_xi(q);
}


void AugmentedRigidArm::update(Vector2Nd q, Vector2Nd dq) {

    // first update xi (augmented model parameters)
    update_xi(q);
    update_Jxi(q);
    update_dJxi(q, dq);

    extract_B_G();
}

void AugmentedRigidArm::extract_B_G() {
    // the fun part- extracting the B_xi(inertia matrix) and G_xi(gravity) from RBDL
    
    // first run ID with dQ and ddQ as zero vectors (gives gravity vector)
    VectorNd dQ_zeros = VectorNd::Zero(NUM_ELEMENTS*6);
    VectorNd ddQ_zeros = VectorNd::Zero(NUM_ELEMENTS*6);
    VectorNd tau = VectorNd::Zero(NUM_ELEMENTS*6);
    InverseDynamics(*rbdl_model, xi, dQ_zeros, ddQ_zeros, tau);
    G_xi = tau;
    
    // next, iterate through by making ddQ_zeros a unit vector and get inertia matrix
    for (int i = 0; i < NUM_ELEMENTS*6; ++i) {
        for (int j = 0; j < NUM_ELEMENTS * 6; ++j) {
            ddQ_zeros(j) = 0.0;
        }
        ddQ_zeros(i) = 1.0;
        InverseDynamics(*rbdl_model, xi, dQ_zeros, ddQ_zeros, tau);
        B_xi.col(i) = tau - G_xi;
    }
}

void AugmentedRigidArm::joint_publish(){

}

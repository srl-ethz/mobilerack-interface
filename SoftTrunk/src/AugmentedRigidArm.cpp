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
    // ros::init() requires argc and argv for remapping, but since we're not using command line arguments for this, input placeholder values that won't be used.
    int tmp_c = 1;
    char *tmp_v[1];
    strcpy(tmp_v[0], "placeholder");
      ros::init(tmp_c, tmp_v, "joint_pub", ros::init_options::AnonymousName);
      std::cout<<"1\n";
      ros::NodeHandle n;
      nodeHandle = &n;
      std::cout<<"2\n";
      joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 10);
      std::cout<<"3\n";
      for (int i=0; i<NUM_ELEMENTS; i++){
        // set up joint names
        jointState.name.push_back(std::to_string(i)+"-base-ball-joint_x_joint");
        jointState.name.push_back(std::to_string(i)+"-base-ball-joint_y_joint");
        jointState.name.push_back(std::to_string(i)+"-base-ball-joint_z_joint");
        jointState.name.push_back(std::to_string(i)+"-a-b_joint");
        jointState.name.push_back(std::to_string(i)+"-middle-ball-joint_x_joint");
        jointState.name.push_back(std::to_string(i)+"-middle-ball-joint_y_joint");
        jointState.name.push_back(std::to_string(i)+"-middle-ball-joint_z_joint");
        jointState.name.push_back(std::to_string(i)+"-c-d_joint");
        jointState.name.push_back(std::to_string(i)+"-tip-ball-joint_x_joint");
        jointState.name.push_back(std::to_string(i)+"-tip-ball-joint_y_joint");
        jointState.name.push_back(std::to_string(i)+"-tip-ball-joint_z_joint");
      }
      for (int i=0; i<NUM_ELEMENTS*11; i++)
        jointState.position.push_back(0.0);
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
      std::cout << "Robot model created, with " << rbdl_model->dof_count << " DoF. It is a " << rbdl_model->dof_count/11 <<"-segment arm.\n";
}

Eigen::Matrix<double, 3, 1> AugmentedRigidArm::straw_bend_joint(double phi, double theta){
  Eigen::Matrix<double, 3, 1> angles = Eigen::Matrix<double, 3, 1>::Zero();
  if (theta==0)
    return angles;
  angles(0) = -atan(tan(theta)*sin(phi));
  angles(1) = asin(sin(theta/2)*cos(phi));
  angles(2) = -asin( (cos(phi)*sin(phi)*cos(theta)-sin(phi)*cos(phi)) / cos(angles(1)));
  return angles;
}

void  AugmentedRigidArm::update_xi(Vector2Nd q) {
    double theta; double phi; double thetaX; double thetaY; double deltaL;
    for (int i = 0; i < NUM_ELEMENTS; ++i) {
        // use phi, theta parametrization because it's simpler for calculation
        if (q(2*i) == 0) {
            if (q(2*i + 1) == 0)
                phi = 0;
            else
                phi = PI / 2;
        }
        else
            phi = atan(q(2*i+1)/q(2*i)-cos(PI/2) /sin(PI/2));
        if(q(2*i) == 0)
            theta = -q(2*i+1)/(TRUNK_RADIUS*cos(PI/2-phi));
        else
            theta = -q(2*i)/(TRUNK_RADIUS*cos(phi));
        if(theta<0){
          theta = -theta;
          phi += PI;
        }
        double b = lengths[i]/2;
        if (theta!=0)
          b = lengths[i]/theta * sqrt(1.0+4.0*sin(theta/2.0)/theta * (sin(theta/2.0)/theta - cos(theta/2.0)));
        double nu = 0;
        if (theta!=0)
          nu = acos(1.0/b*lengths[i]/theta*sin(theta/2.0));

        deltaL = lengths[i]/2.0 - b;
        xi.block(11*i+0,0,3,1) = straw_bend_joint(phi, theta/2-nu);
        xi(11*i+3) = deltaL;
        xi.block(11*i+4,0,3,1) = straw_bend_joint(phi, 2*nu);
        xi(11*i+7) = deltaL;
        xi.block(11*i+8,0,3,1) = xi.block(11*i+0,0,3,1);
    }
}

void AugmentedRigidArm::update_Jxi(Vector2Nd q) {
    // brute force calculate Jacobian lol
    //todo: verify that this numerical method is actually okay
    // this particular method only works because xi of each element is totally independent of other elements
    Vector2Nd q_deltaX = Vector2Nd(q);
    Vector2Nd q_deltaY = Vector2Nd(q);
    double epsilon = 0.0001;
    for (int i = 0; i < NUM_ELEMENTS; ++i) {
        q_deltaX(2*i) += epsilon;
        q_deltaY(2*i+1) += epsilon;
    }
    update_xi(q);
    Eigen::Matrix<double, 11*NUM_ELEMENTS, 1> xi_current = Eigen::Matrix<double, 11*NUM_ELEMENTS, 1>(xi);
    update_xi(q_deltaX);
    Eigen::Matrix<double, 11*NUM_ELEMENTS, 1> xi_deltaX = Eigen::Matrix<double, 11*NUM_ELEMENTS, 1>(xi);
    update_xi(q_deltaY);
    Eigen::Matrix<double, 11*NUM_ELEMENTS, 1> xi_deltaY = Eigen::Matrix<double, 11*NUM_ELEMENTS, 1>(xi);
//    std::cout<<"xi_current"<<xi_current<<"\n";
//    std::cout<<"xi_deltaX"<<xi_deltaX<<"\n";
//    std::cout<<"xi_deltaY"<<xi_deltaY<<"\n";
    for (int j = 0; j < NUM_ELEMENTS; ++j) {
      Jxi.block(11*j, 2*j+0, 11, 1) = (xi_deltaX-xi_current).block(11*j,0,11,1)/epsilon;
        Jxi.block(11*j, 2*j+1, 11, 1) = (xi_deltaY-xi_current).block(11*j,0,11,1)/epsilon;
    }
}

void AugmentedRigidArm::update_dJxi(Vector2Nd q, Vector2Nd dq) {
    //todo: verify this numerical method too
    //todo: actually this messes up Jxi so not using this for now...
    double epsilon=0.1;
    Vector2Nd q_delta = Vector2Nd(q);
    q_delta +=  dq*epsilon;
//    std::cout<<q_delta<<"\n";
    update_Jxi(q);
    Eigen::Matrix<double, 11*NUM_ELEMENTS, 2*NUM_ELEMENTS> Jxi_current = Eigen::Matrix<double, 11*NUM_ELEMENTS, 2*NUM_ELEMENTS>(Jxi);
    update_Jxi(q_delta);
    Eigen::Matrix<double, 11*NUM_ELEMENTS, 2*NUM_ELEMENTS> Jxi_delta = Eigen::Matrix<double, 11*NUM_ELEMENTS, 2*NUM_ELEMENTS>(Jxi);
    dJxi = (Jxi_delta - Jxi_current)/epsilon;
}


void AugmentedRigidArm::update(Vector2Nd q, Vector2Nd dq) {

    // first update xi (augmented model parameters)
    update_xi(q);
    joint_publish();
    update_Jxi(q);
    update_dJxi(q, dq);

    update_xi(q);
    extract_B_G();
}

void AugmentedRigidArm::extract_B_G() {
    // the fun part- extracting the B_xi(inertia matrix) and G_xi(gravity) from RBDL
    
    // first run ID with dQ and ddQ as zero vectors (gives gravity vector)
    VectorNd dQ_zeros = VectorNd::Zero(NUM_ELEMENTS*11);
    VectorNd ddQ_zeros = VectorNd::Zero(NUM_ELEMENTS*11);
    VectorNd tau = VectorNd::Zero(NUM_ELEMENTS*11);
    InverseDynamics(*rbdl_model, xi, dQ_zeros, ddQ_zeros, tau);
    G_xi = tau;
    
    // next, iterate through by making ddQ_zeros a unit vector and get inertia matrix
    for (int i = 0; i < NUM_ELEMENTS*11; ++i) {
        ddQ_zeros(i) = 0.1;
        InverseDynamics(*rbdl_model, xi, dQ_zeros, ddQ_zeros, tau);
        B_xi.col(i) = (tau - G_xi)/ddQ_zeros(i);
        ddQ_zeros(i) = 0;
    }
}

AugmentedRigidArm::~AugmentedRigidArm(){
  ros::shutdown();

}

void AugmentedRigidArm::joint_publish(){
  if (!USE_ROS)
    return;
  jointState.header.stamp = ros::Time::now();
  for(int i=0; i<NUM_ELEMENTS*11; i++)
    jointState.position[i] = xi(i);
  joint_pub.publish(jointState);
  //  std::cout<<"publishing"<<jointState<<"\n";
}

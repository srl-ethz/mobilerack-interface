#ifndef ARMPCC_H
#define ARMPCC_H


#include <rbdl/rbdl.h>
#include "SoftTrunk_common_defs.h"

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>

#include <iostream>
#include <fstream>

#if USE_ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#endif

/**
 * how many joints there are in one segment of the augmented rigid arm.
 */
#define JOINTS 11

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

/**
 * @brief Represents the augmented rigid arm model.
 * @details The rigid arm model  approximates the soft arm. (see paper etc. for more info on how this is so)
 * This class can calculate the kinematics & dynamics of the augmented arm using RBDL.
 * It can also generate XACRO files based on the parameters of the robot. This should then be converted to URDF with ./create_urdf.sh
 */
class AugmentedRigidArm {
private:
    Model *rbdl_model;

    /**
     * @brief create a RBDL model and save it to member variable rbdl_model
     */
    void create_rbdl_model();

    /**
     * @brief extract inertia matrix(B) and gravity vector(G) of the current arm configuration(xi).
     */
    void extract_B_G();

    /**
     * @brief generate a file robot.urdf.xacro, using the lengths and masses of the actual robot.
     */
    void create_xacro();

#if USE_ROS
    ros::Publisher joint_pub;
    sensor_msgs::JointState jointState;
    ros::NodeHandle *nodeHandle;
#endif

    /**
     * @brief convert phi-theta bend to joint angles for a straw-bend joint.
     */
    Eigen::Matrix<double, 3, 1> straw_bend_joint(double phi, double theta);

    void update_m(Vector2Nd);

    void update_Jm(Vector2Nd);

    void update_dJm(Vector2Nd, Vector2Nd);
    /**
    * @brief publish joint state to ROS
    */
    void joint_publish();

public:
    /**
     * @param is_create_xacro set to true if you only want to generate the model's xacro model
     */
    explicit AugmentedRigidArm(bool is_create_xacro = false);

    ~AugmentedRigidArm();

    std::vector<double> lengths = LENGTHS;
    std::vector<double> masses = MASSES;
    Eigen::Matrix<double, NUM_ELEMENTS * JOINTS, 1> m;
    /**
     * @brief the Jacobian that maps from q to xi
     */
    Eigen::Matrix<double, NUM_ELEMENTS * JOINTS, NUM_ELEMENTS * 2> Jm = Eigen::Matrix<double,
            NUM_ELEMENTS * JOINTS, NUM_ELEMENTS * 2>::Zero(); // Jacobian
    /**
     * @brief the time derivative of the Jacobian that maps from q to xi
     */
    Eigen::Matrix<double, NUM_ELEMENTS * JOINTS, NUM_ELEMENTS * 2> dJm = Eigen::Matrix<double, NUM_ELEMENTS * JOINTS, NUM_ELEMENTS * 2>::Zero(); // time derivative of Jacobian
    void update_Jxi(Vector2Nd q); // used for inverse kinematics
    Eigen::Matrix<double, 3, NUM_ELEMENTS * JOINTS> Jxi = Eigen::Matrix<double, 3, NUM_ELEMENTS * JOINTS>::Zero();

    /**
     * @brief inertia matrix
     */
    Eigen::Matrix<double, NUM_ELEMENTS * JOINTS, NUM_ELEMENTS * JOINTS> B_xi = Eigen::Matrix<double,
            NUM_ELEMENTS * JOINTS, NUM_ELEMENTS * JOINTS>::Zero();
    /**
     * @brief the gravity vector, i.e. the force at each joint when the arm is completely stationary at its current configuration.
     */
    Eigen::Matrix<double, NUM_ELEMENTS * JOINTS, 1> G_xi = Eigen::Matrix<double, NUM_ELEMENTS * JOINTS, 1>::Zero();

    /**
     * @brief update the member variables based on current values
     */
    void update(Vector2Nd, Vector2Nd);


};

#endif

#pragma once

#include <drake/common/drake_assert.h>
#include <drake/common/find_resource.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include "drake/systems/analysis/simulator.h"
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>

#include "SoftTrunk_common.h"

#if USE_ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#endif

/**
 * how many joints there are in one segment of the augmented rigid arm.
 */
#define JOINTS 11

/**
 * @brief Represents the augmented rigid arm model.
 * @details The rigid arm model  approximates the soft arm. (see paper etc. for more info on how this is so)
 * This class can calculate the kinematics & dynamics of the augmented arm using RBDL.
 */
class AugmentedRigidArm {
private:
    /** @brief this builder helps in adding & connecting system blocks */
    drake::systems::DiagramBuilder<double> builder;
    /** @brief SceneGraph manages all the systems that uses geometry
     * https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_scene_graph.html
     * */
    drake::geometry::SceneGraph<double> &scene_graph = *builder.AddSystem<drake::geometry::SceneGraph>();
    /** @brief MultibodyPlant is the model of interconnected bodies.
     * https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html
     */
    drake::multibody::MultibodyPlant<double> *multibody_plant = builder.AddSystem<drake::multibody::MultibodyPlant<double>>(
            1.0e-3);
    std::unique_ptr<drake::systems::Diagram<double>> diagram;
    std::unique_ptr<drake::systems::Context<double>> diagram_context;
    // drake::geometry::DrakeVisualizerParams drakevisualizer_params{};


    /**
     * @brief load from URDF and set up Drake model
     */
    void setup_drake_model();

    /**
     * @brief extract inertia matrix(B) and gravity vector(G) of the current arm configuration(xi).
     */
    void extract_B_G();

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

    void update_Jxi(Vector2Nd q);

    /**
    * @brief publish joint state to ROS
    */
    void joint_publish();

public:
    /**
     * @param is_create_xacro set to true if you only want to generate the model's xacro model
     */
    explicit AugmentedRigidArm();

    ~AugmentedRigidArm();

    /**
     * @brief m is the map from q to the augmented model's parameters
     */
    Eigen::Matrix<double, N_SEGMENTS * JOINTS, 1> m;
    /**
     * @brief the Jacobian that maps from q to xi
     */
    Eigen::Matrix<double, N_SEGMENTS * JOINTS, N_SEGMENTS * 2> Jm = Eigen::Matrix<double,
            N_SEGMENTS * JOINTS, N_SEGMENTS * 2>::Zero(); // Jacobian
    /**
     * @brief the time derivative of the Jacobian that maps from q to xi
     */
    Eigen::Matrix<double, N_SEGMENTS * JOINTS, N_SEGMENTS * 2> dJm = Eigen::Matrix<double,
            N_SEGMENTS * JOINTS, N_SEGMENTS * 2>::Zero(); // time derivative of Jacobian

    Eigen::Matrix<double, 3, N_SEGMENTS * JOINTS> Jxi = Eigen::Matrix<double, 3, N_SEGMENTS * JOINTS>::Zero();

    /**
     * @brief inertia matrix
     */
    Eigen::Matrix<double, N_SEGMENTS * JOINTS, N_SEGMENTS * JOINTS> B_xi = Eigen::Matrix<double,
            N_SEGMENTS * JOINTS, N_SEGMENTS * JOINTS>::Zero();
    /**
     * @brief the gravity vector, i.e. the force at each joint when the arm is completely stationary at its current configuration.
     */
    Eigen::Matrix<double, N_SEGMENTS * JOINTS, 1> G_xi = Eigen::Matrix<double, N_SEGMENTS * JOINTS, 1>::Zero();

    /**
     * @brief update the member variables based on current values
     */
    void update(Vector2Nd, Vector2Nd);

    /** @brief simulate the rigid body model in Drake. The prismatic joints are broken... */
    void simulate();
};

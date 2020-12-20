/**
 * @file SoftTrunk_common.h
 * @brief defines various constants and convenience functions for the Soft Trunk that are used across different files.
 */

#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <thread>
#include <assert.h>
#include <array>

using namespace Eigen;

/** @brief which type of parametrization is used to describe the PCC configuration. */
enum class ParametrizationType {
    phi_theta /** as used in Katzschmann2019 */,
    longitudinal /** see yasu's report & DellaSantina2020 */
    };
/** @brief the link structure of the rigid model */
enum class RigidModelType {
    original /** as proposed in Katzschmann2019 */,
    straw_bend /** see yasu's report */
    };
/** @brief placement of arm */
enum class ArmConfigurationType {
    stalactite /** hanging from ceiling- stick "tight" to the ceiling */,
    stalagmite /** placed on floor- rise "might"ily from the floor */
    };

/** @brief robot-specific parameters SoftTrunk*/
namespace st_params {
    const std::string robot_name = "3segment_4chamber";
    /** @brief mass of each segment, in kg */
    std::array<double, 3> masses = {0.12, 0.12, 0.12};
    /** @brief length of each segment, in m */
    std::array<double, 3> lengths = {0.11, 0.11, 0.11};
    const int num_segments = 3;

    const std::string local_address = "192.168.1.111";

    /** @brief baseline pressure of arm. The average of the pressures sent to a segment should be this pressure.
     * for DragonSkin 30, set to 300.
     * for DragonSkin 10, set to 150.
     * (not throughly examined- a larger or smaller value may be better)
    */
    const int p_offset = 150;

    /** @brief radius of the soft trunk, in meters. */
    const double r_trunk = 0.03;

    /** @brief valve-related parameters */
    namespace valve {
        /** @brief IP address of Festo valves */
        const char* address = "192.168.0.100";
        /** @brief map[i] is the valve number for i-th actuator in controller */
        std::array<int, 4> map = {1,2,3,4};
        const int num_valves = 16;

        /** @brief max pressure that can be sent out. Useful to prevent puncture of the arm with too high a pressure.
        * for DragonSkin 30, set to 1200.
        * for DragonSkin 10, set to 400.
        * (not throughly examined- a larger or smaller value may be better)
        */
        const int max_pressure = 400;
        const bool use_pid = false;
        const bool log = true;
    }
    /** @brief qualisys-related parameters */
    namespace qualisys{
        const char* address = "172.17.126.97";
        const unsigned short port = 22222;
    }

    const ParametrizationType parametrization = ParametrizationType::phi_theta;
    const RigidModelType rigidModel = RigidModelType::straw_bend;
    const ArmConfigurationType armConfiguration = ArmConfigurationType::stalactite;
}

void sleep(double sleep_secs) {
    std::this_thread::sleep_for(std::chrono::milliseconds((int) (sleep_secs * 1000)));
}
/**
 * @brief how many PCC elements there are
 */
#define N_SEGMENTS 2
/**
 * @brief number of chambers in a single segment. 3 or 4 is supported.
 */
#define N_CHAMBERS 4
/**
 * @brief IP address of this computer
 */
#define LOCAL_ADDRESS "192.168.1.111"

#define PI 3.141592
/**
 * @brief period of one control step, in seconds. must be a value longer than the control loop.
 */
#define CONTROL_PERIOD 0.005

/**
 * @brief defines a matrix of size (NUM_ELEMENTS*2,1), for convenience
 */
typedef Eigen::Matrix<double, N_SEGMENTS * 2, 1> Vector2Nd;
/**
 * @brief defines a matrix of size (NUM_ELEMENTS*2,NUM_ELEMENTS*2), for convenience
 */
typedef Eigen::Matrix<double, N_SEGMENTS * 2, N_SEGMENTS * 2> Matrix2Nd;

// set up ROS automatically if ROS is found
#ifdef CATKIN_FOUND
#define USE_ROS true  // do you want to publish joint states to ROS
#endif
#ifndef CATKIN_FOUND
#define USE_ROS false
#endif

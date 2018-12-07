/**
 * @file SoftTrunk_common_defs.h
 * @brief defines various definitions for the Soft Trunk that are used across different files.
 */

#ifndef SOFTTRUNK_SOFTTRUNK_COMMON_DEFS_H
#define SOFTTRUNK_SOFTTRUNK_COMMON_DEFS_H


#include <Eigen/Dense>
/**
 * @brief how many PCC elements there are
 */
#define NUM_ELEMENTS 3
/**
 * @brief Used in characterization- how many time steps to spend on each bend
 */
#define CHARACTERIZE_STEPS 20
/**
 * @brief IP address of this computer
 */
#define LOCAL_ADDRESS "192.168.1.111"
/**
 * @brief IP address of computer running Motive
 */
#define MOTIVE_ADDRESS "192.168.1.105"
/**
 * @brief IP address of Festo valves
 */
#define VALVE_ADDRESS "192.168.1.101"
/**
 * @brief baseline pressure of arm. The average of the pressures sent to a segment should be this pressure.
 */
#define PRESSURE_OFFSET 700
/**
 * @brief define max pressure that can be sent out. Useful to prevent puncture of the arm with too high a pressure.
 */
#define MAX_PRESSURE 1400
/**
 * @brief arm is moved with PID controller, not the dynamic controller, if true.
 */
#define USE_PID_CURVATURE_CONTROL false
#define USE_FEEDFORWARD_CONTROL false
/**
 * @brief number of chambers in a single segment. 3 or 4 is supported.
 */
#define CHAMBERS 4
/**
 * @brief radius of the soft trunk, in meters.
 */
#define TRUNK_RADIUS 0.03
#define PI 3.141592
/**
 * @brief period of one control step, in seconds. must be above 0.002??(verify) because that's how long a control loop takes
 */
#define CONTROL_PERIOD 0.005

typedef Eigen::Matrix<double,NUM_ELEMENTS*2,1> Vector2Nd;
typedef Eigen::Matrix<double,NUM_ELEMENTS*2,NUM_ELEMENTS*2> Matrix2Nd;


#endif //SOFTTRUNK_SOFTTRUNK_COMMON_DEFS_H

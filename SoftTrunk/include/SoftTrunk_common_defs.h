//
// Created by yasu on 29/10/18.
//

#ifndef SOFTTRUNK_SOFTTRUNK_COMMON_DEFS_H
#define SOFTTRUNK_SOFTTRUNK_COMMON_DEFS_H


/*
 * defines common parameters, typedefs used across source files.
 */

#include <Eigen/Dense>
#define NUM_ELEMENTS 3 // how many PCC elements there are
#define CHARACTERIZE_STEPS 20 // Used in characterization- how many time steps to spend on each bend
#define LOCAL_ADDRESS "192.168.1.111"
#define MOTIVE_ADDRESS "192.168.1.105"
#define VALVE_ADDRESS "192.168.1.101"
#define PRESSURE_OFFSET 700 // baseline pressure of arm.
#define MAX_PRESSURE 1400
#define USE_PID_CURVATURE_CONTROL false
#define USE_FEEDFORWARD_CONTROL false
#define CHAMBERS 4 // 3 or 4 is supported.
#define TRUNK_RADIUS 0.03 // radius of trunk, in meters
#define PI 3.141592
#define CONTROL_PERIOD 0.005 // period of one control step, in seconds. must be above 0.002??(verify) because that's how long a control loop takes


typedef Eigen::Matrix<double,NUM_ELEMENTS*2,1> Vector2Nd;
typedef Eigen::Matrix<double,NUM_ELEMENTS*2,NUM_ELEMENTS*2> Matrix2Nd;


#endif //SOFTTRUNK_SOFTTRUNK_COMMON_DEFS_H

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
#define TIME_STEP 100 // milliseconds of each time step
#define LOCAL_ADDRESS "192.168.1.194"
#define MOTIVE_ADDRESS "192.168.1.105"
#define VALVE_ADDRESS "192.168.1.101"
#define PRESSURE_OFFSET 100 // baseline pressure of arm.

typedef Eigen::Matrix<double,NUM_ELEMENTS*2,1> Vector2Nd;
typedef Eigen::Matrix<double,NUM_ELEMENTS*2,NUM_ELEMENTS*2> Matrix2Nd;


#endif //SOFTTRUNK_SOFTTRUNK_COMMON_DEFS_H

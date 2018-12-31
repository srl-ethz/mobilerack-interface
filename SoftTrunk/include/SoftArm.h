//
// Created by yasu on 29/10/18.
//

#ifndef SOFTTRUNK_SOFTARM_H
#define SOFTTRUNK_SOFTARM_H

#include "ForceController.h"
#include "CurvatureCalculator.h"
#include <Eigen/Dense>
#include "SoftTrunk_common_defs.h"

/**
 * @brief Represents the physical soft trunk robot.
 * Gets the current q of robot continuously in the background (using CurvatureCalculator)
 * Outputs pressure to robot
 * Manage physical parameters of arm derived from characterization (such as k, d, alpha)
 */
class SoftArm {
private:
    std::vector<int> valve_map = VALVE_MAP;
    /**
     * @brief used for 3-chamber arm. Provides a mapping matrix from generalized pressure expressed with two parameters to pressure expressed with 3 chambers.
     */
    Eigen::Matrix<double, 3, 2> force_map_matrix;
    ForceController *forceController;
    bool simulate;
public:
    /**
     *
     * @param sim true if running simulation, in which case it does not try to connect to actual arm
     */
    SoftArm(bool sim = false);

    /**
     * actuate the arm
     * @param tau torque,in q space
     */
    void actuate(Vector2Nd tau);

    /**
     * send the pressures to ForceController
     * @param pressures generalized pressure values for each chamber (pressure along x y directions), 0 is neutral
     * todo: elaborate on this in separate document
     */
    void actuatePressure(Vector2Nd pressures); // actuate using pressures for each chamber

    void stop();

    CurvatureCalculator *curvatureCalculator;

    Vector2Nd d = Vector2Nd::Zero();
    Vector2Nd alpha = Vector2Nd::Zero();
    Vector2Nd k = Vector2Nd::Zero();

};

#endif //SOFTTRUNK_SOFTARM_H

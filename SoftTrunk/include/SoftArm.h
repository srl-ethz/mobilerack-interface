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
    void actuate(Vector2Nd tau, bool setAlphaToOne=false);
    /**
     * @brief Provides a mapping matrix from force to pressure.
     */
    Eigen::Matrix<double, N_CHAMBERS, 2> A_f2p;
    Eigen::Matrix<double, 2, N_CHAMBERS> A_p2f;
    /**
    * @brief extends A_p2f to all segments. Used in characterization.
    */
    Eigen::Matrix<double, 2*N_SEGMENTS, N_CHAMBERS*N_SEGMENTS> A_p2f_all;
    /**
     * send the pressures to ForceController
     * @param pressures pressure values for each chamber
     * todo: elaborate on this in separate document
     */
    void actuatePressure(Eigen::Matrix<double, N_SEGMENTS*N_CHAMBERS, 1> pressures); // actuate using pressures for each chamber

    void stop();

    CurvatureCalculator *curvatureCalculator;

    Vector2Nd d = Vector2Nd::Zero();
    Vector2Nd k = Vector2Nd::Zero();
    Vector2Nd alpha;
    /**
     * records pressure of each chamber.
     */
    Eigen::Matrix<double, N_SEGMENTS*N_CHAMBERS, 1> p;

};

#endif //SOFTTRUNK_SOFTARM_H

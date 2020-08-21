//
// Created by yasu on 29/10/18.
//

#ifndef SOFTTRUNK_SOFTARM_H
#define SOFTTRUNK_SOFTARM_H

#include "ValveController.h"
#include "CurvatureCalculator.h"
#include <Eigen/Dense>
#include "SoftTrunk_common_defs.h"

/**
 * @brief Represents the physical soft trunk robot.
 * Gets the current q of robot continuously in the background (using CurvatureCalculator)
 * Outputs pressure to robot
 * Manage physical parameters of arm derived from characterization (such as k, d, alpha)
 */
class SoftTrunkInterface {
private:
    std::vector<int> valve_map = VALVE_MAP;
    ValveController *forceController;
    bool simulate;
    /**
     * @brief Provides a mapping matrix from pressure to force for a single segment.
     */
    Eigen::Matrix<double, N_CHAMBERS, 2> A_f2p;
    /**
     * @brief Provides a mapping matrix from force to pressure for a single segment.
     */
    Eigen::Matrix<double, 2, N_CHAMBERS> A_p2f;

public:
    /**
     *
     * @param sim true if running simulation, in which case it does not try to connect to actual arm
     */
    SoftTrunkInterface(bool sim = false);

    /**
     * actuate the arm
     * @param f force values
     * @param setAlphaToOne set alpha=1. Used in the PID controller so that output can directly map to pressure
     */
    void actuate(Vector2Nd f, bool setAlphaToOne=false);
    /**
     * send the pressures to ForceController
     * @param pressures pressure values for each chamber
     */
    void actuatePressure(Eigen::Matrix<double, N_SEGMENTS*N_CHAMBERS, 1> pressures); // actuate using pressures for each chamber
    /**
    * @brief mapping matrix from force to pressure for a single segment. (extends A_p2f to all segments)
    */
    Eigen::Matrix<double, 2*N_SEGMENTS, N_CHAMBERS*N_SEGMENTS> A_p2f_all;
    ~SoftTrunkInterface();

    CurvatureCalculator *curvatureCalculator;
    /**
     * current pressure commanded to each chamber.
     */
    Eigen::Matrix<double, N_SEGMENTS*N_CHAMBERS, 1> p;
    Vector2Nd d = Vector2Nd::Zero();
    Vector2Nd k = Vector2Nd::Zero();
    Vector2Nd alpha;
};

#endif //SOFTTRUNK_SOFTARM_H

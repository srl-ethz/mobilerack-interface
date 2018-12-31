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
 */
class SoftArm {
    /*
     * class that acts as interface for I/O of physical soft arm (curvatures, pressures etc.) and combines the soft arm's parameters(like k,d)
     */
private:
    std::vector<int> valve_map = VALVE_MAP;
    /**
     * @brief used for 3-chamber arm. Provides a map from force expressed in La, Lb to the same force expressed with 3 chambers.
     */
    Eigen::Matrix<double, 3, 2> force_map_matrix;
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
    CurvatureCalculator *curvatureCalculator;
    ForceController *forceController;

    void stop();

//    Vector2Nd k;
    Vector2Nd d;
    Vector2Nd alpha = Vector2Nd::Zero();;
    Vector2Nd k = Vector2Nd::Zero();
    bool simulate;
};

#endif //SOFTTRUNK_SOFTARM_H

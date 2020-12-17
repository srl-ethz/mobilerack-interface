//
// Created by yasu on 26/10/18.
//

#ifndef SOFTTRUNK_CONTROLLERPCC_H
#define SOFTTRUNK_CONTROLLERPCC_H

#include "SoftTrunk_common_defs.h"
#include <Eigen/Dense>
#include <AugmentedRigidArm.h>
#include "SoftTrunkInterface.h"
#include "MiniPID.h"


/**
 * @brief Implements the PCC controller as described in paper.
 * @details It receives pointers to instances of AugmentedRigidArm and SoftArm, so it can access instances of those classes to retrieve information about them that can be used in the Manager class to control the Soft Trunk.
 * By setting USE_PID_CURVATURE_CONTROL to true in SoftTrunk_common_defs.h, it can also do PID control.
 */
class ControllerPCC {
public:
    /**
     *
     * @param augmentedRigidArm pointer to instance of AugmentedRigidArm.
     * @param softTrunkInterface pointer to instance of SoftArm.
     */
    ControllerPCC(AugmentedRigidArm *augmentedRigidArm, SoftTrunkInterface *softTrunkInterface, bool use_feedforward=false, bool simulate = false);

    /**
     * compute the torque required to actuate the arm with a dynamic controller.
     * Implements the controller described in the paper.
     * @param q_ref
     * @param dq_ref
     * @param ddq_ref
     * @param f pointer to where you want the torque value to be saved.
     */
    void curvatureDynamicControl(
            const Vector2Nd &q_ref,
            const Vector2Nd &dq_ref,
            const Vector2Nd &ddq_ref,
            Vector2Nd *f);

    /**
     * compute the pressures for good old PID control.
     * @param q_ref target configuration
     * @param pressures pointer to where you want the pressures values to be saved.
     */
    void curvaturePIDControl(
            const Vector2Nd &q_ref,
            Vector2Nd *pressures
    );
    /**
     * @brief update B(inertia matrix), C and G(gravity vector) in q space
     */
    void updateBCG(const Vector2Nd &q, const Vector2Nd &dq);
    Matrix2Nd B;
    Matrix2Nd C;
    Vector2Nd G;
    Eigen::Matrix<double, 3, 2*N_SEGMENTS> J;

private:
    AugmentedRigidArm *ara;
    SoftTrunkInterface *sti;
    std::vector<MiniPID> miniPIDs;
    bool use_feedforward;
    bool simulate;

};


#endif //SOFTTRUNK_CONTROLLERPCC_H

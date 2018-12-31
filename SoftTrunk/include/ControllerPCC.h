//
// Created by yasu on 26/10/18.
//

#ifndef SOFTTRUNK_CONTROLLERPCC_H
#define SOFTTRUNK_CONTROLLERPCC_H

#include "SoftTrunk_common_defs.h"
#include <Eigen/Dense>
#include <AugmentedRigidArm.h>
#include "SoftArm.h"
#include "MiniPID.h"


/**
 * @brief Implements the PCC controller as described in paper.
 * @details It receives pointers to instances of AugmentedRigidArm and SoftArm, so it can access instances of those classes to retrieve information about them that can be used in the Manager class to control the Soft Trunk.
 * By setting USE_PID_CURVATURE_CONTROL to true in SoftTrunk_common_defs.h, it can also do PID control.
 */
class ControllerPCC{
public:
    /**
     *
     * @param augmentedRigidArm pointer to instance of AugmentedRigidArm.
     * @param softArm pointer to instance of SoftArm.
     */
    ControllerPCC(AugmentedRigidArm *augmentedRigidArm, SoftArm *softArm);
    /**
     * compute the torque required to actuate the arm.
     * @param q_ref
     * @param dq_ref
     * @param ddq_ref
     * @param tau pointer to where you want the torque value to be saved.
     */
    void curvatureDynamicControl(
            const Vector2Nd &q_ref,
            const Vector2Nd &dq_ref,
            const Vector2Nd &ddq_ref,
            Vector2Nd *tau,
            bool simulate=false);

    /**
     * compute the torque for good old PID control.
     * @param q_ref
     * @param pressures pointer to where you want the pressures values to be saved.
     */
    void curvaturePIDControl(
            const Vector2Nd &q_ref,
            Vector2Nd *pressures
            );
    /**
     * @brief update B(inertia matrix), C and G(gravity vector) in q space
     */
    void updateBCG(const Vector2Nd &q, const Vector2Nd &dq); // update B(inertia matrix), C and G(gravity vector)
    Matrix2Nd B;
    Matrix2Nd C;
    Vector2Nd G;
private:
  AugmentedRigidArm *ara;
  SoftArm *sa;
  std::vector<MiniPID> miniPIDs;
  Vector2Nd phi_PD_control(Vector2Nd);

};



#endif //SOFTTRUNK_CONTROLLERPCC_H

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



class ControllerPCC{
    /*
     * Implements the PCC controller as described in paper.
     */
public:
    ControllerPCC(AugmentedRigidArm *, SoftArm *);
    void curvatureDynamicControl(
            const Vector2Nd &q_ref,
            const Vector2Nd &dq_ref,
            const Vector2Nd &ddq_ref,
            Vector2Nd *tau); // pass on the measured & reference values, to get tau.

    void curvatureDynamicControl(
            const Vector2Nd &q_meas,
            const Vector2Nd &dq_meas,
            const Vector2Nd &q_ref,
            const Vector2Nd &dq_ref,
            const Vector2Nd &ddq_ref,
            Vector2Nd *tau); // pass on the measured & reference values, to get tau
    void curvaturePIDControl(
            const Vector2Nd &q_ref,
            Vector2Nd *output
            );
private:
  Matrix2Nd B;
  Matrix2Nd C;
  Vector2Nd G;
  AugmentedRigidArm *ara;
  SoftArm *sa;
  std::vector<MiniPID> miniPIDs;
};



#endif //SOFTTRUNK_CONTROLLERPCC_H

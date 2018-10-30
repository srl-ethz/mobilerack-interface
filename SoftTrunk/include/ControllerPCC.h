//
// Created by yasu on 26/10/18.
//

#ifndef SOFTTRUNK_CONTROLLERPCC_H
#define SOFTTRUNK_CONTROLLERPCC_H

#define NUM_ELEMENTS 3

#include <Eigen/Dense>
#include <AugmentedRigidArm.h>

typedef Eigen::Matrix<double,NUM_ELEMENTS*2,1> Vector2Nd;
typedef Eigen::Matrix<double,NUM_ELEMENTS*2,NUM_ELEMENTS*2> Matrix2Nd;

class ControllerPCC{
    /*
     * Implements the PCC controller as described in paper.
     */
public:
    ControllerPCC(Vector2Nd, Vector2Nd, AugmentedRigidArm *); // pass on k and d and pointer to AugmentedRigidArm
    void curvatureDynamicControl(
            const Vector2Nd &q_meas,
            const Vector2Nd &dq_meas,
            const Vector2Nd &q_ref,
            const Vector2Nd &dq_ref,
            const Vector2Nd &ddq_ref,
            Vector2Nd *tau); // pass on the measured & reference values, to get tau.
private:
  Vector2Nd k;
  Vector2Nd d;
  Matrix2Nd B;
  Matrix2Nd C;
  Vector2Nd G;
  AugmentedRigidArm *ara;
};



#endif //SOFTTRUNK_CONTROLLERPCC_H

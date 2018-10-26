//
// Created by yasu on 26/10/18.
//

#ifndef SOFTTRUNK_CONTROLLERPCC_H
#define SOFTTRUNK_CONTROLLERPCC_H

#define NUM_ELEMENTS 3

#include <Eigen/Dense>
#include <AugmentedRigidArm.h>

class ControllerPCC{
    /*
     * Implements the PCC controller as described in paper.
     */
public:
    ControllerPCC(Eigen::Matrix<double, NUM_ELEMENTS*2, NUM_ELEMENTS*2>, Eigen::Matrix<double, NUM_ELEMENTS*2, NUM_ELEMENTS*2>, AugmentedRigidArm*); // pass on K and D and pointer to AugmentedRigidArm
    void curvatureDynamicControl(
            Eigen::Matrix<double,NUM_ELEMENTS*2,1> q_meas,
            Eigen::Matrix<double,NUM_ELEMENTS*2,1> dq_meas,
            Eigen::Matrix<double,NUM_ELEMENTS*2,1> q_ref,
            Eigen::Matrix<double,NUM_ELEMENTS*2,1> dq_ref,
            Eigen::Matrix<double,NUM_ELEMENTS*2,1> ddq_ref,
            Eigen::Matrix<double,NUM_ELEMENTS*2,1> &tau); // pass on the measured & reference values, to get tau.
private:
    Eigen::Matrix<double, NUM_ELEMENTS*2, NUM_ELEMENTS*2> K;
    Eigen::Matrix<double, NUM_ELEMENTS*2, NUM_ELEMENTS*2> D;
    Eigen::Matrix<double, NUM_ELEMENTS*2, NUM_ELEMENTS*2> B;
    Eigen::Matrix<double, NUM_ELEMENTS*2, NUM_ELEMENTS*2> C;
    Eigen::Matrix<double, NUM_ELEMENTS*2, 1> G;
    AugmentedRigidArm* ara;
};



#endif //SOFTTRUNK_CONTROLLERPCC_H

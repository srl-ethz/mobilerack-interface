//
// Created by yasu on 26/10/18.
//

#pragma once

#include "SoftTrunk_common.h"
#include <AugmentedRigidArm.h>
#include "ValveController.h"
#include "CurvatureCalculator.h"
#include "MiniPID.h"

/**
 * @brief Implements the PCC controller as described in paper.
 * @details It receives pointers to instances of AugmentedRigidArm and SoftArm, so it can access instances of those classes to retrieve information about them that can be used in the Manager class to control the Soft Trunk.
 * By setting USE_PID_CURVATURE_CONTROL to true in SoftTrunk_common_defs.h, it can also do PID control.
 * @todo it is in the midst of conversion process to new system, doesn't work for now.
 */
class ControllerPCC {
public:
    ControllerPCC();

    /**
     * compute the torque required to actuate the arm with a dynamic controller.
     * Implements the controller described in the paper.
     * @param q_ref
     * @param dq_ref
     * @param ddq_ref
     */
    void curvatureDynamicControl(
            const VectorXd &q_ref,
            const VectorXd &dq_ref,
            const VectorXd &ddq_ref);

    /**
     * compute the pressures for good old PID control.
     * @param q_ref target configuration
     * @param pressures pointer to where you want the pressures values to be saved.
     */
    void curvaturePIDControl(
            const VectorXd &q_ref,
            VectorXd *pressures
    );
    /**
     * @brief update B(inertia matrix), C and G(gravity vector) in q space
     */
    void updateBCG(const VectorXd &q, const VectorXd &dq);
    MatrixXd B;
    MatrixXd C;
    VectorXd G;
    MatrixXd J; // Eigen::Matrix<double, 3, 2*N_SEGMENTS>

private:
    std::unique_ptr<AugmentedRigidArm> ara;
    std::unique_ptr<ValveController> vc;
    std::unique_ptr<CurvatureCalculator> cc;

    VectorXd k; /** @brief stiffness coefficient of silicone arm */ 
    VectorXd d; /** @brief damping coefficient of silicone arm */
    VectorXd alpha;

    /**
     * @brief Provides a mapping matrix from pressure to force for a single segment.
     */
    MatrixXd A_f2p;
    /**
     * @brief Provides a mapping matrix from force to pressure for a single segment.
     */
    MatrixXd A_p2f;
    MatrixXd A_p2f_all;
    
    std::vector<MiniPID> miniPIDs;
    bool use_feedforward;
    bool simulate;
    /**
     * actuate the arm
     * @param f force values
     */
    void actuate(VectorXd f);

};
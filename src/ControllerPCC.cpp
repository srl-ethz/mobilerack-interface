//
// Created by yasu and rkk on 26/10/18.
//

#include "ControllerPCC.h"

/**
 * @brief implements a PID controller whose parameters are defined using the Ziegler-Nichols method.
 * @param Ku ultimate gain
 * @param period oscillation period (in seconds)
 * @return MiniPID controller
 */
MiniPID ZieglerNichols(double Ku, double period) {
    // https://en.wikipedia.org/wiki/Ziegler–Nichols_method
//    double Kp = 0.2 * Ku;
//    double Ki = Kp / (period / 2.0) * CONTROL_PERIOD;
//    double Kd = Kp * period / 3.0 / CONTROL_PERIOD;
    double Kp = 0.45*Ku;
    double Ki = Kp / (period/1.2) * CONTROL_PERIOD;
    double Kd = 0.;
    return MiniPID(Kp, Ki, Kd);
}

ControllerPCC::ControllerPCC(){
    std::cout<<"ControllerPCC created...\n";
    // set up PID controllers
    for (int j = 0; j < st_params::num_segments*2; ++j) {
        miniPIDs.push_back(ZieglerNichols(30000,0.36));
    }
    assert(st_params::num_segments == 2);
    k = VectorXd::Zero(st_params::num_segments*2);
    d = VectorXd::Zero(st_params::num_segments*2);
    alpha = VectorXd::Zero(st_params::num_segments*2);
    A_f2p = MatrixXd::Zero(4, 2);
    A_p2f = MatrixXd::Zero(2, 4);
    A_p2f_all = MatrixXd::Zero(2*st_params::num_segments, 4*st_params::num_segments);

    A_f2p << 0.5, 0., 0., 0.5, -0.5, 0., 0., -0.5;
    A_p2f << 1., 0., -1., 0., 0., 1., 0., -1.;
    for (int j = 0; j < st_params::num_segments; ++j) {
        A_p2f_all.block(2*j, 4*j, 2, 4) = A_p2f;
    }

    // set up the impedance parameters (k&d), and actuation coefficient(alpha).
    k(0) = 340;
    k(1) = k(0);
    k(2) = 230;
    k(3) = k(2);
    for (int l = 0; l < st_params::num_segments * 2; ++l)
        d(l) = 27;
    alpha(0) = 0.00988;
    alpha(1) = alpha(0);
    alpha(2) = 0.0076;
    alpha(3) = alpha(2);

    ara = std::make_unique<AugmentedRigidArm>();
    vc = std::make_unique<ValveController>();
    cc = std::make_unique<CurvatureCalculator>();

}

void ControllerPCC::curvatureDynamicControl(const VectorXd &q_ref,
                                            const VectorXd &dq_ref,
                                            const VectorXd &ddq_ref) {
    // variables to save the measured values.
    VectorXd q_meas;
    VectorXd dq_meas;
    if (use_feedforward or simulate) {
        // don't use the actual values, since it's doing feedforward control.
        q_meas = q_ref;
        dq_meas = dq_ref;
    } else {
        // get the current configuration from SoftTrunkInterface.
        q_meas = cc->q;
        dq_meas = cc->dq;
    }
    updateBCG(q_meas, dq_meas);
    VectorXd f = k.asDiagonal() * q_ref + d.asDiagonal()* dq_ref + G + C * dq_ref + B * ddq_ref;
}

void ControllerPCC::updateBCG(const VectorXd &q, const VectorXd &dq) {
    ara->update(q, dq);
    // these conversions from m space to q space are described in the paper
    B = ara->Jm.transpose() * ara->B_xi * ara->Jm;
    C = ara->Jm.transpose() * ara->B_xi * ara->dJm;
    G = ara->Jm.transpose() * ara->G_xi;
    J = ara->Jxi * ara->Jm;
}

void ControllerPCC::curvaturePIDControl(const VectorXd &q_ref, VectorXd *pressures) {
    // for (int i = 0; i < 2 * N_SEGMENTS; ++i) {
    //     (*pressures)(i) = miniPIDs[i].getOutput(sti->curvatureCalculator->q(i),q_ref(i));
    // }
}

void ControllerPCC::actuate(VectorXd f) {
    VectorXd mappedPressure = VectorXd::Zero(st_params::num_segments * 4);
    double tmp_min0;
    double tmp_min1;
    // @todo: figure out what's going on here....
    for (int j = 0; j < st_params::num_segments; ++j) {
        mappedPressure.segment(4 * j, 4) = (A_f2p * f.segment(2 * j, 2))/alpha(2*j);
        for (int l = 0; l < 4; ++l) {
            mappedPressure(4*j+l) += st_params::p_offset; // add pressure offset to each
        }
        tmp_min0 = std::min(0.0, std::min(mappedPressure(4*j+0), mappedPressure(4*j+2)));
        tmp_min1 = std::min(0.0, std::min(mappedPressure(4*j+1), mappedPressure(4*j+3)));
        for (int l = 0; l < 2; ++l) {
            mappedPressure(4*j+l) -= tmp_min0;
            mappedPressure(4*j+1+l) -= tmp_min1;
        }
    }
    for (int l = 0; l < 4 * st_params::num_segments; ++l) {
        vc->setSinglePressure(l, mappedPressure(l));
    }
}
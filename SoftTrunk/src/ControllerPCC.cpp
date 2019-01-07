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

ControllerPCC::ControllerPCC(AugmentedRigidArm *augmentedRigidArm, SoftTrunkInterface *softTrunkInterface, bool use_feedforward, bool simulate) : ara(augmentedRigidArm),
                                                                                       sti(softTrunkInterface), use_feedforward(use_feedforward), simulate(simulate) {
    std::cout<<"ControllerPCC created...\n";
    // set up PID controllers
    for (int j = 0; j < N_SEGMENTS*2; ++j) {
        miniPIDs.push_back(ZieglerNichols(30000,0.36));
    }
}

void ControllerPCC::curvatureDynamicControl(const Vector2Nd &q_ref,
                                            const Vector2Nd &dq_ref,
                                            const Vector2Nd &ddq_ref,
                                            Vector2Nd *f) {
    // variables to save the measured values.
    Vector2Nd q_meas;
    Vector2Nd dq_meas;
    if (use_feedforward or simulate) {
        // don't use the actual values, since it's doing feedforward control.
        q_meas = Vector2Nd(q_ref);
        dq_meas = Vector2Nd(dq_ref);
    } else {
        // get the current configuration from SoftTrunkInterface.
        q_meas = sti->curvatureCalculator->q;
        dq_meas = sti->curvatureCalculator->dq;
    }
    updateBCG(q_meas, dq_meas);
    *f = sti->k.asDiagonal() * q_ref + sti->d.asDiagonal() * dq_ref + G + C * dq_ref + B * ddq_ref;
}

void ControllerPCC::updateBCG(const Vector2Nd &q, const Vector2Nd &dq) {
    ara->update(q, dq);
    // these conversions from m space to q space are described in the paper
    B = ara->Jm.transpose() * ara->B_xi * ara->Jm;
    C = ara->Jm.transpose() * ara->B_xi * ara->dJm;
    G = ara->Jm.transpose() * ara->G_xi;
    J = ara->Jxi * ara->Jm;
}

void ControllerPCC::curvaturePIDControl(const Vector2Nd &q_ref, Vector2Nd *pressures) {
    for (int i = 0; i < 2 * N_SEGMENTS; ++i) {
        (*pressures)(i) = miniPIDs[i].getOutput(sti->curvatureCalculator->q(i),q_ref(i));
    }
}

//
// Created by yasu and rkk on 26/10/18.
//

#include "ControllerPCC.h"

MiniPID ZieglerNichols(double Ku, double period){
    // https://en.wikipedia.org/wiki/Ziegler–Nichols_method
    double Kp = 0.33 * Ku;
    double Ki = Kp/ (period / 2.0) * CONTROL_PERIOD;
    double Kd = Kp * period/3 / CONTROL_PERIOD;
    return MiniPID(Kp, Ki, Kd);
}

ControllerPCC::ControllerPCC(AugmentedRigidArm* augmentedRigidArm, SoftArm* softArm) : ara(augmentedRigidArm), sa(softArm){
    if (USE_PID_CURVATURE_CONTROL){
        miniPIDs.push_back(MiniPID(700,0,0));// PID for phi. Z-N doesn't seem to work very well, so just doing P control...
        miniPIDs.push_back(ZieglerNichols(500, 0.6)); // PID for theta
        miniPIDs.push_back(MiniPID(0,0,0)); // PID for phi
        miniPIDs.push_back(MiniPID(0,0,0)); // PID for theta
        miniPIDs.push_back(MiniPID(0,0,0)); // PID for phi
        miniPIDs.push_back(MiniPID(0,0,0)); // PID for theta
    }
    else{
        miniPIDs.push_back(MiniPID(1.2,0,0.002/CONTROL_PERIOD)); // PID for phi0
        miniPIDs.push_back(MiniPID(1.1,0,0.002/CONTROL_PERIOD)); // PID for phi1
        miniPIDs.push_back(MiniPID(1.0,0,0.002/CONTROL_PERIOD)); // PID for phi2
    }
}


void ControllerPCC::curvatureDynamicControl(const Vector2Nd &q_ref,
                                            const Vector2Nd &dq_ref,
                                            const Vector2Nd &ddq_ref,
                                            Vector2Nd *tau) {

    curvatureDynamicControl(sa->curvatureCalculator->q, sa->curvatureCalculator->dq, q_ref, dq_ref, ddq_ref, tau);
}

Vector2Nd ControllerPCC::phi_PD_control(Vector2Nd q_ref){
    //todo: what does this do?? It outputs PD control for phi which is imperative in doing control of arm
    Vector2Nd result = Vector2Nd::Zero();
    for (int j = 0; j < NUM_ELEMENTS; ++j) {
        result(2*j) = miniPIDs[j].getOutput(sa->curvatureCalculator->q(2*j), q_ref(2*j));
        if (q_ref(2*j+1) < 0.1) // no point in doing phi_PD_control if theta is small
            result(2*j) = 0;
    }

    return result;
}

void ControllerPCC::curvatureDynamicControl(
        const Vector2Nd &q_meas,
        const Vector2Nd &dq_meas,
        const Vector2Nd &q_ref,
        const Vector2Nd &dq_ref,
        const Vector2Nd &ddq_ref,
        Vector2Nd *tau) {
    ara->update(q_meas, dq_meas);
    B = ara->Jm.transpose() * ara->B_xi * ara->Jm;
    C = ara->Jm.transpose() * ara->B_xi * ara->dJm;
    G = ara->Jm.transpose() * ara->G_xi;


    *tau = sa->k.asDiagonal()*q_ref + sa->d.asDiagonal()*dq_ref + G + C*dq_ref + B*ddq_ref + phi_PD_control(q_ref);
}

void ControllerPCC::curvaturePIDControl(const Vector2Nd &q_ref, Vector2Nd *output) {
    for (int i = 0; i < 2 * NUM_ELEMENTS; ++i) {
        (*output)(i) = miniPIDs[i].getOutput(sa->curvatureCalculator->q(i), q_ref(i));
    }
}

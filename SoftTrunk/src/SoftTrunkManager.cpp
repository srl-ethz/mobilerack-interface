//
// Created by yasu on 26/10/18.
//

#include "SoftTrunkManager.h"

SoftTrunkManager::SoftTrunkManager(bool logMode): logMode(logMode) {
    // set up CurvatureCalculator, AugmentedRigidArm, and ControllerPCC objects.
    softArm = new SoftArm{};
    softArm->start();
    augmentedRigidArm = new AugmentedRigidArm{};
    controllerPCC = new ControllerPCC{augmentedRigidArm, softArm};

    logBeginTime = std::chrono::high_resolution_clock::now();
}

void SoftTrunkManager::curvatureControl(Vector2Nd q,
                                        Vector2Nd dq,
                                        Vector2Nd ddq) {
    // get current measured state from CurvatureCalculator inside SoftArm, send that to ControllerPCC
    // actuate the arm with the tau value.

    // sanitize q before sending to controller
    for (int j = 0; j < NUM_ELEMENTS; ++j) {
        if (q(2*j+1) < 0){
            q(2*j) += 3.1415;
            q(2*j+1) = -q(2*j+1);
        }
        q(2*j) = fmod(q(2*j), 3.1415*2);
    }

    if (USE_PID_CURVATURE_CONTROL){
        Vector2Nd output;
        controllerPCC->curvaturePIDControl(q,&output);
        softArm->actuate(output, q);
    }
    else {
        Vector2Nd tau;
        controllerPCC->curvatureDynamicControl(q, dq, ddq, &tau);
        softArm->actuate(tau, q);
    }
    if(logMode)
        log(softArm->curvatureCalculator->q, q);
}

void SoftTrunkManager::log(Vector2Nd &q_meas, Vector2Nd &q_ref) {
    log_q_meas.push_back(q_meas);
    log_q_ref.push_back(q_ref);
    log_time.push_back(std::chrono::high_resolution_clock::now() - logBeginTime);
    logNum++;
}

void SoftTrunkManager::characterize() {
    std::cout << "SoftTrunkManager.characterize called. Computing characteristics of the SoftTrunk...\n";

    // first, specify the pressures to send to arm.
    // the pressure profile has CHARACTERIZE_STEPS*NUM_ELEMENTS*4 steps, and is structured as
    // { actuate +x of first element for CHARACTERIZE_STEPS steps,
    // actuate -x of first element for CHARACTERIZE_STEPS steps,
    // actuate +y of first element for CHARACTERIZE_STEPS steps,
    // actuate -y of first element for CHARACTERIZE_STEPS steps, ...}
    Eigen::Matrix<double, NUM_ELEMENTS*2, CHARACTERIZE_STEPS*NUM_ELEMENTS*4>  pressures = Eigen::Matrix<double, NUM_ELEMENTS*2, CHARACTERIZE_STEPS*NUM_ELEMENTS*4>::Zero();
    for (int i = 0; i < NUM_ELEMENTS*4; ++i) {
        for (int j = 0; j < CHARACTERIZE_STEPS; ++j) {
            if (i%2 == 0){
                pressures(i/2, i*CHARACTERIZE_STEPS + j) = 300;
            }
            else{
                pressures(i/2, i*CHARACTERIZE_STEPS + j) = -300;
            }
        }
    }

    Eigen::Matrix<double, NUM_ELEMENTS*2, CHARACTERIZE_STEPS*NUM_ELEMENTS*4> q_history;
    Eigen::Matrix<double, NUM_ELEMENTS*2, CHARACTERIZE_STEPS*NUM_ELEMENTS*4> dq_history;

    // also log the output as well, for reference
    logMode = true;
    Vector2Nd empty_vec = Vector2Nd::Zero();


    // send that to arm and save the results.
    for (int l = 0; l < CHARACTERIZE_STEPS*NUM_ELEMENTS*4; ++l) {
        softArm->actuatePressure(pressures.col(l));
        std::this_thread::sleep_for(std::chrono::milliseconds(TIME_STEP));
        q_history.col(l) = softArm->curvatureCalculator->q;
        dq_history.col(l) = softArm->curvatureCalculator->dq;
        log(softArm->curvatureCalculator->q, empty_vec);
    }

    // convert q_history to dq_history to matrix that's
    // http://eigen.tuxfamily.org/index.php?title=FAQ#Is_there_a_method_to_compute_the_.28Moore-Penrose.29_pseudo_inverse_.3F
}

SoftTrunkManager::~SoftTrunkManager() {
    softArm->stop();
    if (logMode)
        outputLog();
}
void SoftTrunkManager::outputLog() {
    std::cout << "Outputting log to log.csv...\n";
    std::ofstream log_file;
    log_file.open("log.csv");

    // first the header row
    log_file << "time(millis)";
    for (int k = 0; k < NUM_ELEMENTS*2; ++k) {
        log_file << ", q_ref[" << k << "]";
    }
    for (int k = 0; k < NUM_ELEMENTS*2; ++k) {
        log_file << ", q_meas[" << k << "]";
    }
    log_file << "\n";

    // log actual data
    for (int j = 0; j < logNum; ++j) {
        log_file << std::chrono::duration_cast<std::chrono::milliseconds>(log_time[j]).count();
        for (int k = 0; k < NUM_ELEMENTS*2; ++k) {
            log_file << ", " <<log_q_ref[j](k);
        }
        for (int k = 0; k < NUM_ELEMENTS*2; ++k) {
            log_file << ", " <<log_q_meas[j](k);
        }
        log_file << "\n";
    }
    log_file.close();
    std::cout<<"log output complete.\n";
}

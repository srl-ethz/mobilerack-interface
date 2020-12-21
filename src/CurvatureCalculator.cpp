#include "CurvatureCalculator.h"


CurvatureCalculator::CurvatureCalculator() {
    // initialize size of arrays that record transforms
    abs_transforms.resize(st_params::num_segments + 1);

    q = VectorXd::Zero(st_params::num_segments * 2);
    dq = VectorXd::Zero(st_params::num_segments * 2);
    ddq = VectorXd::Zero(st_params::num_segments * 2);

    setupQualisys();

    calculatorThread = std::thread(&CurvatureCalculator::calculator_loop, this);

    // get the current q, to subtract from measurements to get rid of the offset
//    fmt::print("Waiting for 2 seconds to wait for the arm to stop swinging, and measure the initial q... \n");
//    sleep(2);
//    initial_q = q;
//    fmt::print("initial q measured:{}. This value is considered the offset and will be subtracted from future measurements.\n", initial_q);
}

void CurvatureCalculator::setupQualisys() {
    optiTrackClient = std::make_unique<QualisysClient>(st_params::num_segments + 1);
}

void CurvatureCalculator::setupIntegratedSensor() {
    // for future.
}


void CurvatureCalculator::calculator_loop() {
    std::ofstream log_file;
    if (st_params::qualisys::log) {
        log_file.open("log_curvature.csv");
        log_file << "timestamp";
    }
    for (int i = 0; i < st_params::num_segments; ++i) {
        assert(st_params::parametrization == ParametrizationType::phi_theta);
        log_file << fmt::format(", phi_{}, theta_{}", i, i);
    }
    log_file << "\n";

    VectorXd prev_q = VectorXd::Zero(q.size());
    VectorXd prev_dq = VectorXd::Zero(dq.size());
    double interval = 0.01; // loop interval
    run = true;
    while (run) {
        // this loop continuously monitors the current state.
        calculateCurvature();
        // todo: is there a smarter algorithm to calculate time derivative, that can smooth out noises?
//        presmooth_dq = (q - prev_q) / interval;
        dq = (q - prev_q) / interval;;// (1 - 0.2) * presmooth_dq + 0.2 * dq;
//        presmooth_ddq = (dq - prev_dq) / interval;
        ddq = (dq - prev_dq) / interval;;//(1 - 0.2) * presmooth_ddq e+ 0.2 * ddq;
        prev_q = q;
        prev_dq = dq;
        if (st_params::qualisys::log) {
            log_file << timestamp;
            for (int i = 0; i < 2 * st_params::num_segments; ++i) {
                log_file << fmt::format(", {}", q(i));
            }
            log_file << "\n";
        }
        sleep(interval);
    }
    if (st_params::qualisys::log)
        log_file.close();
}

double sign(double val) {
    if (val == 0) return 0.0;
    else if (val > 0) return 1.0;
    else return -1.0;
}

void CurvatureCalculator::calculateCurvature() {
    // first, update the internal data for transforms of each frame
    optiTrackClient->getData(abs_transforms, timestamp);
    MatrixXd matrix;
    double phi, theta;
    // next, calculate the parameters
    for (int i = 0; i < st_params::num_segments; i++) {
        matrix = (abs_transforms[i].inverse() * abs_transforms[i + 1]).matrix();
        phi = atan2(matrix(1, 3), matrix(0, 3)); // -PI/2 ~ PI/2
        theta = acos(matrix(2, 2));
        if (st_params::parametrization == ParametrizationType::phi_theta) {
            q(2 * i) = phi;
            q(2 * i + 1) = theta;
        } else if (st_params::parametrization == ParametrizationType::longitudinal) {
            q(2 * i) = -st_params::r_trunk * cos(phi) *
                       theta; // deltaLa (the difference in the length of La compared to neutral state)
            q(2 * i + 1) = -st_params::r_trunk * cos(PI / 2 - phi) * theta; // deltaLb
        }
    }
    // q -= initial_q; // implement better way to get rid of initial error...
}

CurvatureCalculator::~CurvatureCalculator() {
    run = false;
    calculatorThread.join();
}

//
// Created by yasu on 26/10/18.
//

#include "SoftTrunkManager.h"

// taken from https://gist.github.com/javidcf/25066cf85e71105d57b6
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

// https://stackoverflow.com/questions/41588159/eigen-matrix-resizing-issue-when-implementing-damped-pseudo-inverse
template<typename Derived>
Derived dampedPinv(const Eigen::MatrixBase<Derived>& a, double rho = 1e-4) {
    return a.transpose() * (a*a.transpose() + rho*rho*Eigen::MatrixBase<Derived>::Identity(a.rows(), a.rows()) ).inverse();
}

SoftTrunkManager::SoftTrunkManager(bool logMode): logMode(logMode) {
    // set up CurvatureCalculator, AugmentedRigidArm, and ControllerPCC objects.
    softArm = new SoftArm{};
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

Eigen::Matrix<double, NUM_ELEMENTS, 1> isolateTheta(Vector2Nd& q){
    Eigen::Matrix<double, NUM_ELEMENTS, 1> q_theta;
    for (int j = 0; j < NUM_ELEMENTS; ++j) {
        q_theta(j) = q(2*j+1);
    }
    return q_theta;
}

void SoftTrunkManager::characterize() {
    std::cout << "SoftTrunkManager.characterize called. Computing characteristics of the SoftTrunk...\n";

    // first, specify the pressures to send to arm.
    // the pressure profile has CHARACTERIZE_STEPS*NUM_ELEMENTS*4 steps, and is structured as
    // { actuate +x of all elements for CHARACTERIZE_STEPS steps,
    // actuate -x of all elements for CHARACTERIZE_STEPS steps,
    // actuate +y of all elements for CHARACTERIZE_STEPS steps,
    // actuate -y of all elements for CHARACTERIZE_STEPS steps, ...}
    // todo: this is a very ugly piece of code (mostly because I implemented finePressures later), clean up later
    const int interpolateSteps = 20;
    Eigen::Matrix<double, NUM_ELEMENTS*2, CHARACTERIZE_STEPS>  pressures = Eigen::Matrix<double, NUM_ELEMENTS*2, CHARACTERIZE_STEPS>::Zero();
    Eigen::Matrix<double, NUM_ELEMENTS*2, CHARACTERIZE_STEPS*interpolateSteps>  finePressures = Eigen::Matrix<double, NUM_ELEMENTS*2, CHARACTERIZE_STEPS*interpolateSteps>::Zero();
    double maxPressure  = 500;
    double pressure;
    for (int k = 0; k < NUM_ELEMENTS; ++k) {
        for (int j = 0; j < CHARACTERIZE_STEPS*interpolateSteps; ++j) {
            pressure= fmin(maxPressure, fmin(maxPressure*((double)j*2/(CHARACTERIZE_STEPS*interpolateSteps)), maxPressure*(2-(double)j*2/(CHARACTERIZE_STEPS*interpolateSteps))));
            finePressures(2*k+1, j) = pressure;
//            finePressures(2*k+1, j+CHARACTERIZE_STEPS*interpolateSteps) = pressure;
        }
    }
//    std::cout <<"pressure profile"<< finePressures <<"\n";
    for (int m = 0; m < CHARACTERIZE_STEPS ; ++m) {
        pressures.col(m) = finePressures.col(m*interpolateSteps);
    }

    Eigen::Matrix<double, NUM_ELEMENTS, CHARACTERIZE_STEPS> q_theta_history; // history of just theta
    Eigen::Matrix<double, NUM_ELEMENTS, CHARACTERIZE_STEPS> dq_theta_history;
    Eigen::Matrix<double, (CHARACTERIZE_STEPS) * NUM_ELEMENTS, 1> f_theta_history;

    // also log the output as well, for reference
    logMode = true;
    Vector2Nd empty_vec = Vector2Nd::Zero();
    Vector2Nd placeHolder = Vector2Nd::Zero(); //not used, just necessary to call controllerPCC->curvatureDynamicControl

    // send that to arm and save the results.
    for (int l = 0; l < CHARACTERIZE_STEPS*interpolateSteps; ++l) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        softArm->actuatePressure(finePressures.col(l));
        if (l%interpolateSteps != 0)
            continue;
        int log_index = l/interpolateSteps;
        q_theta_history.col(log_index) = isolateTheta(softArm->curvatureCalculator->q);
        dq_theta_history.col(log_index) = isolateTheta(softArm->curvatureCalculator->dq);
        controllerPCC->curvatureDynamicControl(empty_vec, empty_vec, empty_vec,&placeHolder); // compute B, C, G
        Vector2Nd f=/*controllerPCC->B * (softArm->curvatureCalculator->ddq) + */controllerPCC->C * softArm->curvatureCalculator->dq + controllerPCC->G; // todo: reincorporate measured ddq to this equation.
        f_theta_history.block(log_index*NUM_ELEMENTS, 0, NUM_ELEMENTS, 1) = isolateTheta(f);
        log(softArm->curvatureCalculator->q, empty_vec);
        if (log_index % CHARACTERIZE_STEPS == 0 and l>0) {
            softArm->actuatePressure(Vector2Nd::Zero());
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }
    }

    // convert pressures, q_theta_history and dq_theta_history to matrix
    Eigen::Matrix<double, 3, (CHARACTERIZE_STEPS) * (NUM_ELEMENTS)> history_matrix;
    for (int l = 0; l < CHARACTERIZE_STEPS; ++l) {
        Vector2Nd pressure=  pressures.col(l);
        history_matrix.block(0, l*NUM_ELEMENTS, 1, NUM_ELEMENTS) = isolateTheta(pressure).transpose();
        history_matrix.block(1, l*NUM_ELEMENTS, 1, NUM_ELEMENTS) = -q_theta_history.col(l).transpose();
        history_matrix.block(2, l*NUM_ELEMENTS, 1, NUM_ELEMENTS) = -dq_theta_history.col(l).transpose();
    }
    std::cout << "first row of history matrix is \n" << history_matrix.row(0) << "\n";
    std::cout << "second row of history matrix is \n" << history_matrix.row(1) << "\n";
    std::cout << "third row of history matrix is \n" << history_matrix.row(2) << "\n";
    std::cout << "f_theta_history is \n" << f_theta_history << "\n";

    // http://eigen.tuxfamily.org/index.php?title=FAQ#Is_there_a_method_to_compute_the_.28Moore-Penrose.29_pseudo_inverse_.3F
    //try ignoring the first series of data, see how it goes
    Eigen::Matrix<double, 3,1> characterization = pseudoinverse(history_matrix.block(0,2*NUM_ELEMENTS, 3, (CHARACTERIZE_STEPS-2)* NUM_ELEMENTS)).transpose() * f_theta_history.block(2*NUM_ELEMENTS,0, NUM_ELEMENTS*(CHARACTERIZE_STEPS-2), 1);
    std::cout<< "characterization is \n"<< characterization <<"\n";

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

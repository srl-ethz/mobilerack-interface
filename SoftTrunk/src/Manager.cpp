//
// Created by yasu on 26/10/18.
//

#include "Manager.h"

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

Manager::Manager(bool logMode): logMode(logMode) {
    // set up CurvatureCalculator, AugmentedRigidArm, and ControllerPCC objects.
    softArm = new SoftArm{};
    augmentedRigidArm = new AugmentedRigidArm{};
    controllerPCC = new ControllerPCC{augmentedRigidArm, softArm};

    logBeginTime = std::chrono::high_resolution_clock::now();
}

void Manager::curvatureControl(Vector2Nd q,
                                        Vector2Nd dq,
                                        Vector2Nd ddq) {
    // get current measured state from CurvatureCalculator inside SoftArm, send that to ControllerPCC
    // actuate the arm with the tau value.

    if (USE_PID_CURVATURE_CONTROL){
        Vector2Nd output;
        controllerPCC->curvaturePIDControl(q,&output);
        softArm->actuate(output);
    }
    else {
        Vector2Nd tau;
        controllerPCC->curvatureDynamicControl(q, dq, ddq, &tau);
        softArm->actuate(tau);
    }
    if(logMode)
        log(softArm->curvatureCalculator->q, q);
}

void Manager::log(Vector2Nd &q_meas, Vector2Nd &q_ref) {
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

void Manager::characterize() {

    std::cout << "Manager.characterize called. Computing characteristics of the SoftTrunk...\n";

    // first, specify the pressures to send to arm.
    // the pressure profile has CHARACTERIZE_STEPS*NUM_ELEMENTS*4 steps, and is structured as
    // { actuate +x of all elements for CHARACTERIZE_STEPS steps,
    // actuate -x of all elements for CHARACTERIZE_STEPS steps,
    // actuate +y of all elements for CHARACTERIZE_STEPS steps,
    // actuate -y of all elements for CHARACTERIZE_STEPS steps, ...}
    // todo: this is a very ugly piece of code (mostly because I implemented finePressures later), clean up!
    // todo: it also doesn't work at its current state for the new configuration, must fix!
    const int interpolateSteps = 20;
    Eigen::Matrix<double, NUM_ELEMENTS*2, CHARACTERIZE_STEPS>  pressures = Eigen::Matrix<double, NUM_ELEMENTS*2, CHARACTERIZE_STEPS>::Zero();
    Eigen::Matrix<double, NUM_ELEMENTS*2, CHARACTERIZE_STEPS*interpolateSteps>  finePressures = Eigen::Matrix<double, NUM_ELEMENTS*2, CHARACTERIZE_STEPS*interpolateSteps>::Zero();
    double maxPressure  = 700; // do not let the pressure saturate
    double pressure;
    for (int k = 0; k < NUM_ELEMENTS; ++k) {
        for (int j = 0; j < CHARACTERIZE_STEPS*interpolateSteps; ++j) {
            pressure= -fmax(100, fmin(maxPressure, fmin(maxPressure*((double)j*2/(CHARACTERIZE_STEPS*interpolateSteps)), maxPressure*(2-(double)j*2/(CHARACTERIZE_STEPS*interpolateSteps)))));
            finePressures(2*k+1, j) = pressure;
        }
    }

    Eigen::Matrix<double, NUM_ELEMENTS, CHARACTERIZE_STEPS> q_theta_history; // history of just theta
    Eigen::Matrix<double, NUM_ELEMENTS, CHARACTERIZE_STEPS> dq_theta_history;
    Eigen::Matrix<double, (CHARACTERIZE_STEPS) * NUM_ELEMENTS, 1> f_theta_history;


    // also log the output as well, for reference
    logMode = true;
    Vector2Nd q = Vector2Nd::Zero();
    Vector2Nd dq = Vector2Nd::Zero();
    Vector2Nd empty_vec = Vector2Nd::Zero(); //not used, just necessary to call controllerPCC->curvatureDynamicControl
    std::chrono::high_resolution_clock::time_point lastTime;
    int duration;

    // send that to arm and save the results.
    for (int l = 0; l < CHARACTERIZE_STEPS*interpolateSteps; ++l) {
        lastTime = std::chrono::high_resolution_clock::now();
        log(softArm->curvatureCalculator->q, empty_vec); // hacking the logging mechanism to log dq as well(designed to only log q)

//        softArm->actuatePressure(finePressures.col(l));
        if (l%interpolateSteps == 0) {
            int log_index = l / interpolateSteps;
            q = softArm->curvatureCalculator->q;
            dq = softArm->curvatureCalculator->dq;
            pressures.col(log_index) = finePressures.col(l);
            q_theta_history.col(log_index) = isolateTheta(q);
            dq_theta_history.col(log_index) = isolateTheta(dq);
            controllerPCC->updateBCG(q, dq); // compute B, C, G
            Vector2Nd f =/*controllerPCC->B * (softArm->curvatureCalculator->ddq) +*/ controllerPCC->C * dq + controllerPCC->G; // todo: reincorporate measured ddq to this equation.
            f_theta_history.block(log_index * NUM_ELEMENTS, 0, NUM_ELEMENTS, 1) = isolateTheta(f);
        }

        // control the loop speed here
        duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - lastTime).count();
        std::this_thread::sleep_for(std::chrono::microseconds(int(std::fmax(10000 - duration - 500, 0))));
    }

    // convert pressures, q_theta_history and dq_theta_history to matrix, for each segment
    Eigen::Matrix<double, 2*NUM_ELEMENTS+1, CHARACTERIZE_STEPS*NUM_ELEMENTS> history_matrix;
    for (int l = 0; l < CHARACTERIZE_STEPS; ++l) {
        Vector2Nd pressure=  pressures.col(l);
        for (int j = 0; j < 2 * NUM_ELEMENTS; ++j) {
            pressure(j) = -pressure(j); // since it's being actuated on the negative pressure side
        }
        history_matrix.block(0, l*NUM_ELEMENTS, NUM_ELEMENTS, NUM_ELEMENTS) = isolateTheta(pressure).asDiagonal();
        history_matrix.block(NUM_ELEMENTS, l*NUM_ELEMENTS, NUM_ELEMENTS, NUM_ELEMENTS) = -1* q_theta_history.col(l).asDiagonal();
        history_matrix.block(2*NUM_ELEMENTS, l*NUM_ELEMENTS, 1, NUM_ELEMENTS) = -1* dq_theta_history.col(l).transpose();
    }
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
//    std::cout<< "history matrix is \n"<<history_matrix.format(CSVFormat)<<"\n";
//    std::cout<< "f_history is \n"<<f_theta_history.format(CSVFormat)<<"\n";
    // http://eigen.tuxfamily.org/index.php?title=FAQ#Is_there_a_method_to_compute_the_.28Moore-Penrose.29_pseudo_inverse_.3F
    //try ignoring the first series of data, see how it goes
    Eigen::Matrix<double, 2*NUM_ELEMENTS+1,1> characterization = pseudoinverse(history_matrix.block(0, NUM_ELEMENTS, 2*NUM_ELEMENTS+1, (CHARACTERIZE_STEPS-1)*NUM_ELEMENTS)).transpose() * f_theta_history.block(NUM_ELEMENTS,0, (CHARACTERIZE_STEPS-1)*NUM_ELEMENTS, 1);
    std::cout<< "characterization is \n"<< characterization <<"\n";
}

Manager::~Manager() {
    softArm->stop();
    if (logMode)
        outputLog();
}
void Manager::outputLog() {
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

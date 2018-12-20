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

void Manager::characterize() {

    std::cout << "Manager.characterize called. Computing characteristics of the SoftTrunk...\n";

    const int historySize = 20; // how many samples to use when calculating (make it too big, and pseudoinverse cannot be calculated)
    const double duration = 10; // for how long the process takes
    const int steps = (int)(duration/CONTROL_PERIOD);
    Eigen::MatrixXd pressures; pressures.resize(NUM_ELEMENTS*CHAMBERS, steps); // https://stackoverflow.com/questions/23414308/matrix-with-unknown-number-of-rows-and-columns-eigen-library
    const double max_output = (MAX_PRESSURE - PRESSURE_OFFSET)*0.6;

    // create pressure profile to send to arm. Pressure is monotonically increased then decreased.
    for (int k = 0; k < NUM_ELEMENTS; ++k) {
        for (int j = 0; j < steps; ++j) {
            pressures(k*CHAMBERS+0, j) = PRESSURE_OFFSET + fmin(max_output, fmin(max_output*((double)j*2/steps), max_output*(2-(double)j*2/steps)));
            pressures(k*CHAMBERS+1, j) = (3* PRESSURE_OFFSET - pressures(k*CHAMBERS+0, j) )/2;
            pressures(k*CHAMBERS+2, j) = pressures(k*CHAMBERS+1, j);
        }
    }

    // log of pressure (take historySize number of samples)
    Eigen::MatrixXd pressure_log; pressure_log.resize(NUM_ELEMENTS*2, historySize);
    // log of q (take historySize number of samples)
    Eigen::MatrixXd q_log; q_log.resize(NUM_ELEMENTS*2, historySize);
    // log of dq (take historySize number of samples)
    Eigen::MatrixXd dq_log; dq_log.resize(NUM_ELEMENTS*2, historySize);

    Vector2Nd initial_q = softArm->curvatureCalculator->q;

    std::chrono::high_resolution_clock::time_point lastTime;
    int loop_time; int log_index=0;

    // send that to arm and save the results.
    for (int l = 0; l < steps; ++l) {
        lastTime = std::chrono::high_resolution_clock::now();
        log(softArm->curvatureCalculator->q, softArm->curvatureCalculator->dq); // hacking the logging mechanism to log dq as well(designed to log commanded q and measured q)
        softArm->actuatePressure(pressures.col(l));
        if (l%(steps/historySize)==1) {
            // log the current state once in a while (to get historySize samples)
            for (int i = 0; i < NUM_ELEMENTS; ++i) {
                // only the first two pressures of each segment is used.
                pressure_log(2*i+0, log_index) = pressures(CHAMBERS*i+0, l)-PRESSURE_OFFSET;
                pressure_log(2*i+1, log_index) = pressures(CHAMBERS*i+1, l)-PRESSURE_OFFSET;
            }
            q_log.col(log_index) = softArm->curvatureCalculator->q;
            dq_log.col(log_index) = softArm->curvatureCalculator->dq;
            log_index ++;
        }
        // control the loop speed here
        loop_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - lastTime).count();
        std::this_thread::sleep_for(std::chrono::microseconds(int(std::fmax(CONTROL_PERIOD*1000000 - loop_time - 500, 0))));
    }

    std::cout<<"pressure_log is \n"<< pressure_log <<"\n";
    std::cout<<"q_log is \n"<<q_log<<"\n";
    std::cout<<"dq_log is \n"<<dq_log<<"\n";

    // when computing, just use the values for first chamber of each segment, and values for La
    // convert the recorded data to matrix
    Eigen::MatrixXd log_matrix;log_matrix.resize(3, historySize*NUM_ELEMENTS);
    // also compute the f for each sample
    Eigen::MatrixXd log_f;log_f.resize(historySize*NUM_ELEMENTS, 1);

    for (int l = 0; l < historySize; ++l) {
        controllerPCC->updateBCG(q_log.col(l), dq_log.col(l));
        Vector2Nd tau= controllerPCC->C*dq_log.col(l) + controllerPCC->G;
        for (int j = 0; j < NUM_ELEMENTS; ++j) {
            log_matrix(0,l*NUM_ELEMENTS+j) = pressure_log(2*j,l);
            log_matrix(1,l*NUM_ELEMENTS+j) = -(q_log(2*j,l) - initial_q(2*j)); // remove offset at beginning
            log_matrix(2,l*NUM_ELEMENTS+j) = -dq_log(2*j,l);
            log_f(l*NUM_ELEMENTS+j) = tau(2*j);
        }
    }
    // outputting to CSV format
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
    std::cout<< "f_history is \n"<<log_f.format(CSVFormat)<<"\n";
    std::cout<< "history matrix is \n"<<log_matrix.format(CSVFormat)<<"\n";

    Eigen::Matrix<double, 3,1> characterization = pseudoinverse(log_matrix).transpose() * log_f;
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

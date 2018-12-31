#ifndef SOFTTRUNK_INCLUDE_CurvatureCalculator_H_
#define SOFTTRUNK_INCLUDE_CurvatureCalculator_H_

#define USE_OPTITRACK 0
#define USE_INTEGRATEDSENSOR 1

#include "OptiTrackClient.h"
#include "SoftTrunk_common_defs.h"

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <thread>
#include <cmath>

/**
 * @brief Calculates the current configuration(deltaLx, deltaLy of each segment) of each soft arm segment based on the OptiTrack measurements of the base and tip of a segment.
 * @details The frames have to be set up properly in Motive first. id conventions: base of robot is 0, frame that is between first and second segment is 1, and so on....
 */
class CurvatureCalculator {
private:
    int sensorType;
    OptiTrackClient *optiTrackClient;
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> abs_transforms; // record data from OptiTrack. Absolute transforms for each frame.
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> rel_transforms; // derived from abs_transforms
    std::thread calculatorThread;

    void calculatorThreadFunction(); // background process for calculating curvature
    bool run;

    void calculateCurvature(); // calculates phi and theta from the current frame values.
    Vector2Nd presmooth_q = Vector2Nd::Zero();
    Vector2Nd presmooth_dq = Vector2Nd::Zero();
    Vector2Nd presmooth_ddq = Vector2Nd::Zero();

    double phi;
    double theta;// used while calculating. phi, theta is the commonly used parametrization in PCC

public:
    /**
     *
     * @param sensorType USE_OPTITRACK or USE_INTEGRATEDSENSOR
     */
    explicit CurvatureCalculator(int sensorType);

    void setupOptiTrack(std::string localAddress, std::string serverAddress);

    /*
    uses the optitrack system to measure curvature.
    id conventions:
    base is 0, first frame after that is 1, and so on...
    */
    void setupIntegratedSensor(); // for future, if you want to use sensors embedded in arm.
    void start(); //calling this starts a thread that continuously calculates theta, phi, and their time derivatives.
    void stop(); // stops the thread, disconnects.

    Vector2Nd q = Vector2Nd::Zero(); // [deltaLa0, deltaLb0, deltaLa1, ...]
    // La, Lb: length of a line along the surface of arm,
    Vector2Nd dq = Vector2Nd::Zero(); // derivative of dq
    Vector2Nd ddq = Vector2Nd::Zero(); // derivative of dq
};

#endif // SOFTTRUNK_INCLUDE_CurvatureCalculator_H_

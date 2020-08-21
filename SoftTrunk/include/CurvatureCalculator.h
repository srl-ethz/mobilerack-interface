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
    /**
     * @brief record data from OptiTrack. Saves absolute transforms for each frame.
     */
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> abs_transforms;
    /**
     * @brief derived from abs_transforms. Saves relative transforms between frames.
     */
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> rel_transforms;
    std::thread calculatorThread;

    /**
     * @brief background process that calculates curvature
     */
    void calculatorThreadFunction();
    /**
     * @brief thread runs while this is true
     */
    bool run;

    /**
     * @brief calculates q from the current frame values.
     */
    void calculateCurvature();

    Vector2Nd initial_q = Vector2Nd::Zero();

    double phi;
    double theta;// used while calculating. phi, theta is the commonly used parametrization in PCC

public:
    /**
     *
     * @param sensorType USE_OPTITRACK or USE_INTEGRATEDSENSOR
     */
    explicit CurvatureCalculator(int sensorType);
    /**
     * @brief     uses the optitrack system to measure curvature.
     * id conventions
     * * base is 0
     * * first frame after that is 1, and so on...
     */
    void setupOptiTrack(std::string localAddress, std::string serverAddress);

    /**
     * @brief for future, if you want to use sensors embedded in arm.
     */
    void setupIntegratedSensor();
    /**
     * @brief calling this starts a thread that continuously calculates q, dq, and ddq
     */
    void start();
    void stop(); // stops the thread, disconnects.

    /**
     * @brief [deltaLa0, deltaLb0, deltaLa1, ...]
     * (La, Lb: length of a line along the surface of arm)
     */
    Vector2Nd q = Vector2Nd::Zero();
    Vector2Nd dq = Vector2Nd::Zero();
    Vector2Nd ddq = Vector2Nd::Zero();
};

#endif // SOFTTRUNK_INCLUDE_CurvatureCalculator_H_

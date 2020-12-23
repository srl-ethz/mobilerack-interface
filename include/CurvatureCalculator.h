#pragma once

#include "QualisysClient.h"
#include "SoftTrunk_common.h"

#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <thread>
#include <cmath>

/**
 * @brief Calculates the PCC configuration of each soft arm segment based on motion track / internal sensor measurement.
 * @details For Qualisys mode, the frames have to be set up properly in QTM (using Rigid Body) first.
 * Rigid Body label conventions: base of robot is 0, tip of first segment is 1, and so on...
 * Z axis of each frame is parallel to lengthwise direction of the arm, and origin of each frame is at center of tip of segment
 * @todo cannot gracefully deal with missed frames etc when there are occlusions.
 */
class CurvatureCalculator {
private:
    std::unique_ptr<QualisysClient> optiTrackClient;
    unsigned long long int timestamp;
    /**
     * @brief recorded data from motion tracking system. Saves absolute transforms for each frame.
     */
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> abs_transforms;
    std::thread calculatorThread;

    /**
     * @brief background process that calculates curvature
     */
    void calculator_loop();
    /**
     * @brief thread runs while this is true
     */
    bool run;

    /**
     * @brief calculates q from the current frame values.
     */
    void calculateCurvature();

    VectorXd initial_q;

public:
    explicit CurvatureCalculator();

    // @todo is this actually called at the end??
    ~CurvatureCalculator();

    void setupQualisys();

    /**
     * @brief for future, if you want to use sensors embedded in arm.
     */
    void setupIntegratedSensor();

    /** @brief PCC configuration of each segment of soft arm. depends on st_params::parametrization */
    VectorXd q;
    VectorXd dq;
    VectorXd ddq;
};

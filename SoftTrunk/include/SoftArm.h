//
// Created by yasu on 29/10/18.
//

#ifndef SOFTTRUNK_SOFTARM_H
#define SOFTTRUNK_SOFTARM_H

#include "ForceController.h"
#include "CurvatureCalculator.h"
#include <Eigen/Dense>
#include "SoftTrunk_common_defs.h"

class SoftArm{
    /*
     * class that acts as interface for I/O of physical soft arm (curvatures, pressures etc.) and combines the soft arm's parameters(like k,d)
     */
private:
    std::vector<int> valve_map = {4, 6, 7, 5, 8, 10, 11, 9, 12, 14,15, 13};//{4,5,6,7,8,9,10,11,12,13,14,15}; // Should be ordered in: {root stage x positive -> root stage x negative -> root stage y positive -> ...}
    std::vector<double> outputPressures;


public:
    SoftArm();
    void start();
    void actuate(Vector2Nd); // input tau in phi-theta coordinates
    void actuatePressure(Vector2Nd); // actuate using pressures
    CurvatureCalculator* curvatureCalculator;
    ForceController* forceController;
    void stop();
    Vector2Nd k;
    Vector2Nd d;
    Vector2Nd alpha;
};

#endif //SOFTTRUNK_SOFTARM_H

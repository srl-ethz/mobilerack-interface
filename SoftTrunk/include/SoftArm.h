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
    std::vector<int> valve_map = {7, 5, 4, 6, 11, 9, 8, 10, 15, 13, 12, 14};// Should be ordered in: {root stage x positive -> root stage x negative -> root stage y positive -> ...}
    std::vector<double> outputPressures;


public:
    SoftArm(bool sim=false); // sim=true if simulation (does not try to connect to actual arm)
    void actuate(Vector2Nd, Vector2Nd); // input tau in phi-theta coordinates
    void actuatePressure(Vector2Nd); // actuate using pressures
    CurvatureCalculator* curvatureCalculator;
    ForceController* forceController;
    void stop();
    Vector2Nd k;
    Vector2Nd d;
    Vector2Nd alpha;
    bool simulate;
};

#endif //SOFTTRUNK_SOFTARM_H

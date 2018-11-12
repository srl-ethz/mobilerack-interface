//
// Created by yasu on 31/10/18.
//

#include "SoftTrunkManager.h"
#include <Eigen/Dense>
#define DURATION 20

#include "SoftTrunk_common_defs.h"

int main(){
    SoftTrunkManager stm{true};
    Vector2Nd q = Vector2Nd::Zero();
    Vector2Nd dq = Vector2Nd::Zero();
    Vector2Nd ddq = Vector2Nd::Zero();
    double seconds;

    for (double seconds  = 0; seconds < DURATION; seconds+=CONTROL_PERIOD) {

        // first set the commands
        // make sure phi are not near 0 (problems when crossing over 0)
        //todo: having phi go over phi=0 without hitch with PID controller
        int a=2;
        q(0) = PI + 1 * cos(a*seconds);
        dq(0) = -1 * a* sin(a*seconds);
        ddq(0) = -1 * a*a* cos(a*seconds);
        double startUpTime = 1;
        double maxTheta = 0.3;
        if (seconds < startUpTime){
            q(1) = maxTheta/2 - maxTheta/2 * cos(seconds*(PI/startUpTime));
            dq(1) = -maxTheta/2 * sin(seconds*(PI/startUpTime)) * (PI/startUpTime);
            ddq(1) = -maxTheta/2 * cos(seconds*(PI/startUpTime))* (PI*PI/(startUpTime*startUpTime));
        }
        else
            q(1) = maxTheta;
        q(2) = PI;
        q(3) = 0.2;

        q(4) = PI;
        q(5) = 0.2;

        stm.curvatureControl(q, dq, ddq);
        std::this_thread::sleep_for(std::chrono::microseconds(int(CONTROL_PERIOD*1000000)));
    }
}
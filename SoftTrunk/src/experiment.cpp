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

    std::chrono::high_resolution_clock::time_point lastTime;
    int duration;
    int i;
    long long sum = 0;

    for (double seconds  = 0; seconds < DURATION; seconds+=CONTROL_PERIOD) {
        i++;
        lastTime = std::chrono::high_resolution_clock::now();
        // first set the commands
        // make sure phi are not near 0 (problems when crossing over 0)
        //todo: having phi go over phi=0 without hitch with PID controller
        int a=2;
        q(0) = PI + 1 * cos(a*seconds);
        dq(0) = -1 * a* sin(a*seconds);
        ddq(0) = -1 * a*a* cos(a*seconds);

        q(2) = PI + 1 * cos(a*seconds+PI/3);
        dq(2) = -1 * a* sin(a*seconds+PI/3);
        ddq(2) = -1 * a*a* cos(a*seconds+PI/3);

        q(4) = PI + 1 * cos(a*seconds+PI*2/3);
        dq(4) = -1 * a* sin(a*seconds+PI*2/3);
        ddq(4) = -1 * a*a* cos(a*seconds+PI*2/3);

        double startUpTime = 1;
        double maxTheta = 0.4;
        if (seconds < startUpTime){
            q(1) = maxTheta/2 - maxTheta/2 * cos(seconds*(PI/startUpTime));
            dq(1) = -maxTheta/2 * sin(seconds*(PI/startUpTime)) * (PI/startUpTime);
            ddq(1) = -maxTheta/2 * cos(seconds*(PI/startUpTime))* (PI*PI/(startUpTime*startUpTime));

            q(3) = maxTheta/2 - maxTheta/2 * cos(seconds*(PI/startUpTime));
            dq(3) = -maxTheta/2 * sin(seconds*(PI/startUpTime)) * (PI/startUpTime);
            ddq(3) = -maxTheta/2 * cos(seconds*(PI/startUpTime))* (PI*PI/(startUpTime*startUpTime));

            q(5) = maxTheta/2 - maxTheta/2 * cos(seconds*(PI/startUpTime));
            dq(5) = -maxTheta/2 * sin(seconds*(PI/startUpTime)) * (PI/startUpTime);
            ddq(5) = -maxTheta/2 * cos(seconds*(PI/startUpTime))* (PI*PI/(startUpTime*startUpTime));
        }
        else {
            q(1) = maxTheta;
            q(3) = maxTheta;
            q(5) = maxTheta;
        }

        stm.curvatureControl(q, dq, ddq);
        duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-lastTime).count();
        sum += duration;
        std::this_thread::sleep_for(std::chrono::microseconds(int(std::fmax(CONTROL_PERIOD*1000000 - duration - 500, 0)))); //todo: properly manage time count
    }
    std::cout << "control loop took on average " << sum/i <<" microseconds.\n";
}
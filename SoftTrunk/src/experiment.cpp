//
// Created by yasu on 31/10/18.
//

#include "Manager.h"
#include <Eigen/Dense>
#define DURATION 6

/**
 * @file experiment.cpp
 * @brief Topmost code to run experiments on the Soft Trunk system.
 */

#include "SoftTrunk_common_defs.h"

int main(){
    Manager stm{true};
    Vector2Nd q = Vector2Nd::Zero();
    Vector2Nd dq = Vector2Nd::Zero();
    Vector2Nd ddq = Vector2Nd::Zero();
    double seconds;

    std::chrono::high_resolution_clock::time_point lastTime;
    int duration;
    int i;
    int experiment_type = 4;
    long long sum = 0;

    double a = 2;
    double maxTheta = 0.3;
    double T=4.0;

    double theta;
    double dtheta;
    double ddtheta;
    double phi;
    double dphi;
    double ddphi;

    for (double seconds = 0; seconds < DURATION; seconds += CONTROL_PERIOD) {
        i++;
        lastTime = std::chrono::high_resolution_clock::now();
        // first set the commands
        // make sure phi are not near 0 (problems when crossing over 0)
        //todo: having phi go over phi=0 without hitch with PID controller

        if (experiment_type == 1) {
            q(0) = PI + 1 * cos(a * seconds);
            dq(0) = -1 * a * sin(a * seconds);
            ddq(0) = -1 * a * a * cos(a * seconds);

            q(2) = PI + 1 * cos(a * seconds + PI / 3);
            dq(2) = -1 * a * sin(a * seconds + PI / 3);
            ddq(2) = -1 * a * a * cos(a * seconds + PI / 3);

            q(4) = PI + 1 * cos(a * seconds + PI * 2 / 3);
            dq(4) = -1 * a * sin(a * seconds + PI * 2 / 3);
            ddq(4) = -1 * a * a * cos(a * seconds + PI * 2 / 3);

            double startUpTime = 1;
            if (seconds < startUpTime) {
                q(1) = maxTheta / 2 - maxTheta / 2 * cos(seconds * (PI / startUpTime));
                dq(1) = -maxTheta / 2 * sin(seconds * (PI / startUpTime)) * (PI / startUpTime);
                ddq(1) = -maxTheta / 2 * cos(seconds * (PI / startUpTime)) * (PI * PI / (startUpTime * startUpTime));

                q(3) = maxTheta / 2 - maxTheta / 2 * cos(seconds * (PI / startUpTime));
                dq(3) = -maxTheta / 2 * sin(seconds * (PI / startUpTime)) * (PI / startUpTime);
                ddq(3) = -maxTheta / 2 * cos(seconds * (PI / startUpTime)) * (PI * PI / (startUpTime * startUpTime));

                q(5) = maxTheta / 2 - maxTheta / 2 * cos(seconds * (PI / startUpTime));
                dq(5) = -maxTheta / 2 * sin(seconds * (PI / startUpTime)) * (PI / startUpTime);
                ddq(5) = -maxTheta / 2 * cos(seconds * (PI / startUpTime)) * (PI * PI / (startUpTime * startUpTime));
            } else {
                q(1) = maxTheta;
                q(3) = maxTheta;
                q(5) = maxTheta;
            }
        }
        else if (experiment_type == 2) {
            q(1) = maxTheta/2 - maxTheta/2 * cos(a * seconds);
            dq(1) = maxTheta/2 * a * sin(a * seconds);
            ddq(1) = maxTheta/2 * a * a * cos(a * seconds);

            q(3) = maxTheta/2 - maxTheta/2 * cos(a * seconds + PI/3);
            dq(3) = maxTheta/2 * a * sin(a * seconds + PI/3);
            ddq(3) = maxTheta/2 * a * a * cos(a * seconds + PI/3);

            q(5) = maxTheta/2 - maxTheta/2 * cos(a * seconds + 2*PI/3);
            dq(5) = maxTheta/2 * a * sin(a * seconds + 2*PI/3);
            ddq(5) = maxTheta/2 * a * a * cos(a * seconds + 2*PI/3);

            q(0) = PI/2;
            q(2) = PI;
            q(4) = 3*PI/2;
        }
        else if (experiment_type==3){
            // draws something like a circle

            if(seconds< T){
                theta = 0.2 - 0.2 * cos(seconds * PI / T);
                dtheta = 0.2 * PI / T *  sin(seconds * PI / T);
                ddtheta = 0.2 * PI / T * PI / T * cos(seconds * PI / T);
            }
            else{
                theta = 0.3 + 0.1 * cos(seconds*2*PI/T);
                dtheta = -0.1 * 2*PI/T * sin(seconds*2*PI/T);
                ddtheta = -0.1 * 2*PI/T * 2*PI/T * cos(seconds*2*PI/T);
            }
            phi = PI + PI/5 * sin(seconds * 2*PI/T);
            dphi = PI/5 * 2*PI/T * cos(seconds * 2*PI/T);
            ddphi = -PI/5 * 2*PI/T * 2*PI/T * sin(seconds * 2*PI/T);
            for (int j = 0; j < NUM_ELEMENTS; ++j) {
                q(2*j) = phi;
                dq(2*j) = dphi;
                ddq(2*j) = ddphi;
                q(2*j+1) = theta;
                dq(2*j+1) = dtheta;
                ddq(2*j+1) = ddtheta;
            }
        }
        else if(experiment_type==4){
            for (int j = 0; j < NUM_ELEMENTS; ++j) {
                q(2*j+1) = 0.5;
                q(2*j) = PI/2;
            }
        }

        stm.curvatureControl(q, dq, ddq);
        duration = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - lastTime).count();
        sum += duration;
        std::this_thread::sleep_for(std::chrono::microseconds(
                int(std::fmax(CONTROL_PERIOD * 1000000 - duration - 500, 0)))); //todo: properly manage time count
    }
    std::cout << "control loop took on average " << sum / i << " microseconds.\n";
}
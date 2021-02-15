/**
 * @file common.h
 * @brief include files and define convenience functions that are used across different files.
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <thread>
#include <assert.h>
#include <array>
#include <vector>

#define PI 3.141592

using namespace Eigen;

namespace srl
{
    void sleep(double sleep_secs)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(sleep_secs * 1000)));
    }

    /** @brief use this to run a loop at a fixed rate, regardless of processing time in between. emulates the ros::Rate class. */
    class Rate
    {
        double hz;
        std::chrono::microseconds step;
        std::chrono::time_point<std::chrono::steady_clock> next;

    public:
        Rate(double hz) : hz(hz)
        {
            step = std::chrono::microseconds((int)(1000000. / hz));
            next = std::chrono::steady_clock::now() + step;
        };

        void sleep()
        {
            std::this_thread::sleep_until(next);
            next += step;
        }
    };
} // namespace srl
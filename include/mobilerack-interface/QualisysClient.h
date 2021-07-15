// Copyright 2018 Yasu
#pragma once

#include "common.h"
#include <mutex>

#include "RTProtocol.h"
#include "RTPacket.h"

#include "fmt/core.h"
#include "fmt/ostream.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

/**
 * @brief wraps the qualisys_cpp_sdk library for easy access to motion tracking data- 6D frames & camera images. NEW: also 3D motion marker tracking allowed. Instantiate this class with "Eigen::Vector3d" for motion markers. 
 * @details Frames (& images) are obtained in a separate thread. 6D frame streaming is based on RigidBodyStreaming.cpp from the qualisys_cpp_sdk repo.
 * - cf: https://docs.qualisys.com/qtm-rt-protocol/
 * - cf: https://github.com/qualisys/qualisys_cpp_sdk/blob/master/RTPacket.cpp
 */
class QualisysClient {
public:
    /**
     * @param num_frames how many frames you want to track. The frame labels should be labeled "0", "1", ..., "<num_frames>" in QTM.
     * Can't read 2 (or more) digit labels for now.
     * @param cameraIDs ID of RGB cameras whose images you want to stream. If argument not given, will not stream images.
     * @param nan_if_missed populate the data for a frame with nan values if the data for that timestep has been missed by the mocap system. If false, just use the previous value.
     * @param frameMode string that can be either 3D or 6D, tracking either separate motion marker 3D positions, or whole 6DoF rigid body frames.
     */
    QualisysClient(int num_frames, std::vector<int> cameraIDs = {}, bool nan_if_missed = false, std::string frameMode = "6D");
    ~QualisysClient();

    /**
     * Get the latest frame data.
     * @param frames vector of frames received from motion track. index corresponds to frame label. position in meters
     * @param timestamp timestamp (microseconds) obtained from QTM. @todo this may not be proper value when num_frames at constructor is 0.
     */
    void getData(std::vector<Eigen::Transform<double, 3, Eigen::Affine>> &frames, unsigned long long int &timestamp);
    void getData(std::vector<Eigen::Vector3d> &frames, unsigned long long int &timestamp);

    /**
     * @brief Get the latest Image received from QTM.
     * @param id ID of camera (index of cameraIDs)
     * @param image image from camera
     */
    void getImage(int id, cv::Mat& image);

private:
    CRTProtocol rtProtocol;
    const int majorVersion = 1;
    const int minorVersion = 19;
    const bool bigEndian = false;
    unsigned short udpPort = 6734;
    bool nan_if_missed;

    std::string frameMode = "6D";

    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> frames6D;
    std::vector<Eigen::Vector3d> frames3D;
    unsigned long long int timestamp;

    const std::vector<int> cameraIDs; /** ID of each camera to stream images from. */
    std::vector<cv::Mat> images; /** store images here */

    std::thread motiontrack_thread;
    std::mutex mtx;

    void motiontrack_loop();

    bool connect_and_setup();
};



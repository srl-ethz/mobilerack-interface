// Copyright 2018 Yasu
#pragma once

#include "common.h"
#include <mutex>

#include "RTProtocol.h"
#include "RTPacket.h"

#include "fmt/core.h"
#include "fmt/ostream.h"

/**
 * @brief wraps the qualisys_cpp_sdk library for easy access to motion tracking data
 * @details Frames are obtained in a separate thread. Based on RigidBodyStreaming.cpp from the qualisys_cpp_sdk repo.
 */
class QualisysClient {
public:
    /**
     * @param num_frames how many _frames you want to track. The frame labels should be "0", "1", ..., "<num_frames>".
     * Can't read 2 (or more) digit labels for now.
     */
    QualisysClient(const char *address, const unsigned short port, int num_frames);

    ~QualisysClient();

    /**
     * Get the latest frame data.
     * @param frames vector of frames received from motion track. index corresponds to frame label.
     * @param timestamp timestamp obtained from QTM
     */
    void getData(std::vector<Eigen::Transform<double, 3, Eigen::Affine>> &frames, unsigned long long int &timestamp);

private:
    CRTProtocol rtProtocol;
    const int majorVersion = 1;
    const int minorVersion = 19;
    const bool bigEndian = false;
    unsigned short udpPort = 6734;

    const char *address;
    const unsigned short port;

    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> frames;
    unsigned long long int timestamp;
    std::thread motiontrack_thread;
    std::mutex mtx;

    void motiontrack_loop();

    bool connect_and_setup();
};

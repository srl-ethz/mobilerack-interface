// Copyright 2018 Yasu
#pragma once

#include <stdio.h>


#include "RTProtocol.h"
#include "RTPacket.h"

#include "fmt/core.h"

#include "SoftTrunk_common.h"

/**
 * @brief receives transform data for each frame from the Qualisys system, which can then be used to calculate the current pose of the robot.
 * @details Acts as a client that communicates with the PC running Qualisys software.
 */
class QualisysClient {
public:
    QualisysClient();

    ~QualisysClient();

    /**
     * Try to get a new frame from the listener.
     * @return Each RigidBody contains the transform data for one frame.
     * * RigidBody.id: integer ID
     * * RigidBody.location: Point3f
     * * RigidBody.orientation: Quaternion4f
     */
//    std::vector<RigidBody> getData();

private:
    CRTProtocol rtProtocol;
    const int majorVersion = 1;
    const int minorVersion = 19;
    const bool bigEndian = false;
    unsigned short udpPort = 6734;

    bool connect();
};


// Copyright 2018 Yasu
#ifndef SOFTTRUNK_INCLUDE_OPTITRACKCLIENT_H_
#define SOFTTRUNK_INCLUDE_OPTITRACKCLIENT_H_

#include <stdio.h>


#include <NatNetLinux/CommandListener.h>
#include <NatNetLinux/FrameListener.h>
#include <NatNetLinux/NatNet.h>
#include "SoftTrunk_common_defs.h"

/**
 * @brief receives transform data for each frame from the OptiTrack system, which can then be used to calculate the current pose of the robot.
 * @details Acts as a client that communicates with the PC running Optitrack software. Uses the NatNetLinux library. Based on example_optitrack.cpp, which is based on SimpleExample.cpp provided as example in NatNetLinux.
 */
class OptiTrackClient {
public:
    OptiTrackClient(std::string localAddress, std::string serverAddress);

    /**
     * @brief disconnect from OptiTrack server.
     * @return
     */
    int stop();

    /**
     * Try to get a new frame from the listener.
     * @return Each RigidBody contains the transform data for one frame.
     * * RigidBody.id: integer ID
     * * RigidBody.location: Point3f
     * * RigidBody.orientation: Quaternion4f
     */
    std::vector<RigidBody> getData();

private:
    FrameListener *frameListener;
    CommandListener *commandListener;
    int sdData;
    int sdCommand;
};

#endif

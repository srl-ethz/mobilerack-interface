// Copyright 2018 Yasu
#ifndef SOFTTRUNK_INCLUDE_OPTITRACKCLIENT_H_
#define SOFTTRUNK_INCLUDE_OPTITRACKCLIENT_H_
/*
 * Based on example_optitrack.cpp, which is based on SimpleExample.cpp provided
 * as example in NatNetLinux.
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <NatNetLinux/CommandListener.h>
#include <NatNetLinux/FrameListener.h>
#include <NatNetLinux/NatNet.h>
#include <boost/program_options.hpp>
#include <iostream>
#include <time.h>

class OptiTrackClient {
  /*
  Acts as a client that communicates with the PC running Optitrack software.
  This class takes care of receiving transform data for each frame, which can
  then be used to calculate the current pose of the robot.
  */
public:
  OptiTrackClient(std::string localAddress, std::string serverAddress);
  int stop();                       // disconnect from OptiTrack server.
  std::vector<RigidBody> getData(); // returns a vector of RigidBodys. Each
                                    // RigidBody contains the transform data for
                                    // one frame
  /*
  RigidBody.id: integer ID
  RigidBody.location: Point3f
  RigidBody.orientation: Quaternion4f
  */

private:
  FrameListener *frameListener;
  CommandListener *commandListener;
  int sdData;
  int sdCommand;
};
#endif

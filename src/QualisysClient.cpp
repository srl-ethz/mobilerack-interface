// Copyright 2018 Yasu
#include "QualisysClient.h"

/*
 * Based on example_optitrack, which in turn is based on SimpleExample.cpp
 * provided as example in NatNetLinux.
 */

QualisysClient::QualisysClient() {
    fmt::print("setting up QualisysClient\n");
    connect();
    fmt::print("finished setup of QualisysClient.\n");
}

bool QualisysClient::connect(){
    // loop until connected to server
    for (int i = 0; i < 10; ++i) {
        rtProtocol.Connect(st_params::qualisys::address, st_params::qualisys::port, &udpPort, majorVersion, minorVersion, bigEndian);
        if (rtProtocol.Connected()){
            fmt::print("connected to Qualisys server at {}\n", st_params::qualisys::address);
            return true;
        }
        fmt::print("error: could not connect to Qualisys server at {}, trying again in 1 second...\n", st_params::qualisys::address);
        sleep(1);
    }
    fmt::print("could not connect to Qualisys server 10 times in a row, aborting...\n");
    return false;
}
//std::vector<RigidBody> QualisysClient::getData() {
//    // Try to get a new frame from the listener.
//    bool valid = false;
//    MocapFrame *frame;
//    while (!valid) {
//        frame = new MocapFrame{frameListener->pop(&valid).first};
//    }
//    //std::cout << frame << '\n';
//    return frame->rigidBodies();
//}

QualisysClient::~QualisysClient() {
    fmt::print("stopping QualisysClient\n");
    rtProtocol.StopCapture();
    rtProtocol.Disconnect();
}

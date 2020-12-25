// Copyright 2018 Yasu
#include "mobilerack-interface/QualisysClient.h"

QualisysClient::QualisysClient(const char *address, const unsigned short port, int num_frames) :
        address(address), port(port) {
    _frames.resize(num_frames); // for base + each segment
    connect_and_setup();
    motiontrack_thread = std::thread(&QualisysClient::motiontrack_loop, this);
    fmt::print("finished setup of QualisysClient.\n");
}

bool QualisysClient::connect_and_setup() {
    fmt::print("trying to connect to Qualisys server at {}...\n", address);
    // loop until connected to server
    for (int i = 0; i < 10; ++i) {
        rtProtocol.Connect(address, port, &udpPort, majorVersion,
                           minorVersion, bigEndian);
        if (rtProtocol.Connected()) {
            fmt::print("connected to Qualisys server at {}\n", address);
            break;
        }
        fmt::print("error: could not connect to Qualisys server at {}, trying again in 1 second...\n",
                   address);
        sleep(1);
    }
    bool dataAvailable = false;
    while (!dataAvailable) {
        if (!rtProtocol.Read6DOFSettings(dataAvailable)) {
            printf("rtProtocol.Read6DOFSettings: %s\n\n", rtProtocol.GetErrorString());
            sleep(1);
            continue;
        }
    }
    while (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, CRTProtocol::cComponent6d)) {
        printf("rtProtocol.StreamFrames: %s\n\n", rtProtocol.GetErrorString());
        sleep(1);
    }
    fmt::print("Starting to stream 6DOF data\n");
    return true;
}

void QualisysClient::motiontrack_loop() {
    CRTPacket::EPacketType packetType;
    float fX, fY, fZ;
    float rotationMatrix[9];

    while (true) {
        if (!rtProtocol.Connected()) {
            fmt::print("disconnected from Qualisys server, attempting to reconnect...\n");
            connect_and_setup();
        }

        if (rtProtocol.ReceiveRTPacket(packetType, true) > 0) {
            if (packetType == CRTPacket::PacketData) {
                CRTPacket *rtPacket = rtProtocol.GetRTPacket();
                for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); ++i) {
                    if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix)) {
                        const char *pTmpStr = rtProtocol.Get6DOFBodyName(i);
                        if (pTmpStr) {
                            // convert the ID to an integer
                            // @todo fix implementation to allow more than 1 digit for ID.
                            int id = pTmpStr[0] - '0';
                            if (0 <= id && id < _frames.size()) {
                                // assign value to each frame
                                _frames[id](0, 3) = fX;
                                _frames[id](1, 3) = fY;
                                _frames[id](2, 3) = fZ;
                                for (int row = 0; row < 3; ++row) {
                                    for (int column = 0; column < 3; ++column) {
                                        // column-major order
                                        _frames[id](row, column) = rotationMatrix[column * 3 + row];
                                    }
                                }
                            }
                        }
                        _timestamp = rtPacket->GetTimeStamp();
                    }
                }
            }
        }
    }
}

void QualisysClient::getData(std::vector<Eigen::Transform<double, 3, Eigen::Affine>> &frames,
                             unsigned long long int &timestamp) {
    frames = _frames;
    timestamp = _timestamp;
}

QualisysClient::~QualisysClient() {
    fmt::print("stopping QualisysClient\n");
    motiontrack_thread.join();
    rtProtocol.StopCapture();
    rtProtocol.Disconnect();
}

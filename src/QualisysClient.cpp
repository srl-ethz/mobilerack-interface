// Copyright 2018 Yasu
#include "mobilerack-interface/QualisysClient.h"

QualisysClient::QualisysClient(const char *address, const unsigned short port, int numframes) :
        address(address), port(port) {
    frames.resize(numframes); // for base + each segment
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

    // camera capture can only be started when the program takes control over QTM
    if(!rtProtocol.TakeControl("gait1"))
        fmt::print("becoming master failed, {}\n", rtProtocol.GetErrorString());

    // set to stream images from camera
    unsigned int nCameraId = 9; // corresponds with QTM
    bool enable = true;
    unsigned int nFormat = CRTPacket::EImageFormat::FormatJPG;
    unsigned int w = 320;
    unsigned int h = 200;
    float fLeftCrop = 0; float fTopCrop = 0;
    float fRightCrop = 1; float fBottomCrop = 1;
    if (rtProtocol.SetImageSettings(nCameraId, &enable, (CRTPacket::EImageFormat*)&nFormat, &w, &h, &fLeftCrop, &fTopCrop, &fRightCrop, &fBottomCrop))
        fmt::print("change image settings succeeded\n");
    else
        fmt::print("change image settings failed, {}\n", rtProtocol.GetErrorString());

    // set to stream 6D frames & images
    std::string str = "Image 6D";
    while (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, str.c_str())) {
        printf("rtProtocol.StreamFrames: %s\n\n", rtProtocol.GetErrorString());
        sleep(1);
    }
    fmt::print("Starting to stream 6DOF data\n");

    if(!rtProtocol.ReleaseControl())
        fmt::print("releasing control failed, {}\n", rtProtocol.GetErrorString());
    return true;
}

void QualisysClient::motiontrack_loop() {
    CRTPacket::EPacketType packetType;
    float fX, fY, fZ;
    float rotationMatrix[9];

    while (true) {
        sleep(0.001);
        std::lock_guard<std::mutex> lock(mtx);
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
                            // @todo better processing for when frame is missed (value becomes nan)
                            int id = pTmpStr[0] - '0';
                            if (0 <= id && id < frames.size() && !std::isnan(fX)) {
                                // assign value to each frame
                                frames[id](0, 3) = fX;
                                frames[id](1, 3) = fY;
                                frames[id](2, 3) = fZ;
                                for (int row = 0; row < 3; ++row) {
                                    for (int column = 0; column < 3; ++column) {
                                        // column-major order
                                        frames[id](row, column) = rotationMatrix[column * 3 + row];
                                    }
                                }
                            }
                        }
                        timestamp = rtPacket->GetTimeStamp();
                    }
                }
                for (unsigned int i = 0; i < rtPacket->GetImageCameraCount(); ++i) {
                    unsigned int w, h;
                    rtPacket->GetImageSize(i, w, h);
                    fmt::print("found image, id:{}\twidth:{}\theight:{}\n", i, w, h);
                }
            }
        }
    }
}

void QualisysClient::getData(std::vector<Eigen::Transform<double, 3, Eigen::Affine>> &frames,
                             unsigned long long int &timestamp) {
    std::lock_guard<std::mutex> lock(mtx);
    frames = this->frames;
    timestamp = this->timestamp;
}

QualisysClient::~QualisysClient() {
    fmt::print("stopping QualisysClient\n");
    motiontrack_thread.join();
    rtProtocol.StopCapture();
    rtProtocol.Disconnect();
}

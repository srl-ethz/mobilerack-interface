// Copyright 2018 Yasu
#include "mobilerack-interface/QualisysClient.h"

QualisysClient::QualisysClient(const char *address, int numframes, bool enable_image) :
        address(address), enable_image(enable_image){
    frames.resize(numframes); // for base + each segment
    connect_and_setup();
    motiontrack_thread = std::thread(&QualisysClient::motiontrack_loop, this);
    fmt::print("finished setup of QualisysClient.\n");
}

bool QualisysClient::connect_and_setup() {
    fmt::print("trying to connect to Qualisys server at {}...\n", address);
    // loop until connected to server
    for (int i = 0; i < 5; ++i) {
        rtProtocol.Connect(address, port, &udpPort, majorVersion,
                           minorVersion, bigEndian);
        if (rtProtocol.Connected()) {
            fmt::print("connected to Qualisys server at {}\n", address);
            break;
        }
        fmt::print("error: could not connect to Qualisys server at {}, trying again in 1 second...\n",
                   address);
        sleep(0.5);
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

    // set up image streaming settings
    // currently hardcoded to cameras 9 & 10
    unsigned int nCameraId = 9; // corresponds with QTM
    unsigned int nFormat = CRTPacket::EImageFormat::FormatJPG;
    unsigned int w = 320;
    unsigned int h = 200;
    float fLeftCrop = 0; float fTopCrop = 0;
    float fRightCrop = 1; float fBottomCrop = 1;
    if (rtProtocol.SetImageSettings(nCameraId, &enable_image, (CRTPacket::EImageFormat*)&nFormat, &w, &h, &fLeftCrop, &fTopCrop, &fRightCrop, &fBottomCrop))
        fmt::print("change image settings for camera {} succeeded\n", nCameraId);
    else
        fmt::print("change image settings for camera {} failed, {}\n", nCameraId, rtProtocol.GetErrorString());
    
    nCameraId = 10;
    if (rtProtocol.SetImageSettings(nCameraId, &enable_image, (CRTPacket::EImageFormat*)&nFormat, &w, &h, &fLeftCrop, &fTopCrop, &fRightCrop, &fBottomCrop))
        fmt::print("change image settings for camera {} succeeded\n", nCameraId);
    else
        fmt::print("change image settings for camera {} failed, {}\n", nCameraId, rtProtocol.GetErrorString());

    // set to stream 6D frames (& images)
    std::string str = "6D";
    if (enable_image)
        str = "Image 6D";
    while (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, str.c_str())) {
        printf("rtProtocol.StreamFrames: %s\n\n", rtProtocol.GetErrorString());
        sleep(1);
    }
    fmt::print("Starting to stream data: {}\n", str);

    if(!rtProtocol.ReleaseControl())
        fmt::print("releasing control failed, {}\n", rtProtocol.GetErrorString());
    return true;
}

void QualisysClient::motiontrack_loop() {
    CRTPacket::EPacketType packetType;
    float fX, fY, fZ;
    float rotationMatrix[9];
    char data[480 * 272 * 8 * 3]; /** @todo don't hardcode array size */
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
                                // Qualisys data is in mm
                                frames[id](0, 3) = fX / 1000.;
                                frames[id](1, 3) = fY / 1000.;
                                frames[id](2, 3) = fZ / 1000.;
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
                    /** @todo this process could probably be made more efficient */
                    unsigned int w, h;
                    rtPacket->GetImageSize(i, w, h);
                    unsigned int image_size = rtPacket->GetImageSize(i);
                    // fmt::print("found image, id:{}\twidth:{}\theight:{}\tsize:{}\n", i, w, h, image_size);
                    rtPacket->GetImage(i, data, image_size);
                    rawImage = cv::Mat(1, image_size, CV_8SC1, (void*) data);
                    if (rtPacket->GetImageCameraId(i) == 9)
                        cv::imdecode(rawImage, cv::IMREAD_COLOR, &image1);
                    else if (rtPacket->GetImageCameraId(i) == 10)
                        cv::imdecode(rawImage, cv::IMREAD_COLOR, &image2);
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

void QualisysClient::getImage(cv::Mat& image1, cv::Mat& image2){
    image1 = this->image1;
    image2 = this->image2;
}

QualisysClient::~QualisysClient() {
    fmt::print("stopping QualisysClient\n");
    motiontrack_thread.join();
    rtProtocol.StopCapture();
    rtProtocol.Disconnect();
}

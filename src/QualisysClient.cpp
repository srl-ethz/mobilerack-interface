// Copyright 2018 Yasu
#include "mobilerack-interface/QualisysClient.h"


QualisysClient::QualisysClient(int numframes, std::vector<int> cameraIDs, std::string frameMode, bool nan_if_missed) : cameraIDs(cameraIDs), frameMode(frameMode), nan_if_missed(nan_if_missed) {
    if (frameMode == "6D") {
        frames6D.resize(numframes); // for base + each segment
    }
    else if (frameMode == "3D") {
        frames3D.resize(numframes); // for base + each segment
    }
    else {
        throw std::runtime_error("Failed to recognize frame type (3D or 6D).\n");
    }
    images.resize(cameraIDs.size());
    connect_and_setup();
    motiontrack_thread = std::thread(&QualisysClient::motiontrack_loop, this);
    fmt::print("finished setup of QualisysClient.\n");
}


bool QualisysClient::connect_and_setup() {
    // find the IP address etc. of an QTM RT server instance on the network
    // refer to https://github.com/qualisys/qualisys_cpp_sdk/blob/master/RTClientExample/Operations.cpp
    if (!rtProtocol.DiscoverRTServer(0, false))
        throw std::runtime_error("couldn't find QTM RT server in the network\n");
    int num = rtProtocol.GetNumberOfDiscoverResponses();

    // just connect to the first one it finds, for now
    fmt::print("{} QTM RT server(s) found, connecting to first one...\n", num);
    unsigned int nAddr;
    unsigned short nBasePort;
    std::string msg;
    rtProtocol.GetDiscoverResponse(0, nAddr, nBasePort, msg);
    char tServerAddr[20]; // "should be long enough to store an IP address" - Yasu, 2021
    // convert to string IP address
    sprintf(tServerAddr, "%d.%d.%d.%d", 0xff & nAddr, 0xff & (nAddr >> 8), 0xff & (nAddr >> 16), 0xff & (nAddr >> 24));
    fmt::print("connecting to QTM RT server at {} : {} msg:[{}]\n", tServerAddr, nBasePort, msg);
    
    rtProtocol.Connect(tServerAddr, nBasePort, &udpPort, majorVersion,
                        minorVersion, bigEndian);
    if (!rtProtocol.Connected())
        throw std::runtime_error("couldn't connect to QTM\n");

    bool dataAvailable = false;
    while (!dataAvailable) {
        if (frameMode == "6D") {
            // Reading 6DoF Bodies
            if (!rtProtocol.Read6DOFSettings(dataAvailable)) {
                printf("rtProtocol.Read6DOFSettings: %s\n\n", rtProtocol.GetErrorString());
                srl::sleep(1);
                continue;
            }
        }
        else if (frameMode == "3D") {
            // Reading 3D Motion Markers
            if (!rtProtocol.Read3DSettings(dataAvailable)) {
                fmt::print("rtProtocol.Read3DSettings: %s\n\n", rtProtocol.GetErrorString());
                srl::sleep(1);
                continue;
            }
        }
        else {
            throw std::runtime_error("Failed to recognize frame type (3D or 6D).\n");
        }
    }

    if (cameraIDs.size() > 0){
        // set up camera image streaming
        // camera capture can only be started when the program takes control over QTM
        if(!rtProtocol.TakeControl("gait1"))
            fmt::print("becoming master failed, {}\n", rtProtocol.GetErrorString());

        // set up image streaming settings for each camera
        bool enable = true;
        unsigned int nFormat = CRTPacket::EImageFormat::FormatJPG;
        unsigned int w = 320;
        unsigned int h = 200;
        float fLeftCrop = 0; float fTopCrop = 0;
        float fRightCrop = 1; float fBottomCrop = 1;
        for (unsigned int nCameraId : cameraIDs)
        {
            if (rtProtocol.SetImageSettings(nCameraId, &enable, (CRTPacket::EImageFormat*)&nFormat, &w, &h, &fLeftCrop, &fTopCrop, &fRightCrop, &fBottomCrop))
                fmt::print("change image settings for camera {} succeeded\n", nCameraId);
            else
                fmt::print("change image settings for camera {} failed, {}\n", nCameraId, rtProtocol.GetErrorString());
        }
    }
    
    // set to stream frames (& images)
    std::string str;
    if (frameMode == "6D") {
        // Reading 6DoF Bodies
        str = "6D";
        if (cameraIDs.size() > 0)
            str = "Image 6D";
    }
    else if (frameMode == "3D") {
        // Reading 3D Motion Markers
        str = "3D";
        if (cameraIDs.size() > 0)
            str = "Image 3D";
    }
    else {
        throw std::runtime_error("Failed to recognize frame type (3D or 6D).\n");
    }


    while (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, str.c_str())) {
        printf("rtProtocol.StreamFrames: %s\n\n", rtProtocol.GetErrorString());
        srl::sleep(1);
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
    cv::Mat rawImage; /** temporarily copy received raw bytes to here */
    while (true) {
        srl::sleep(0.001);
        std::lock_guard<std::mutex> lock(mtx);
        if (!rtProtocol.Connected()) {
            // fmt::print("disconnected from Qualisys server, attempting to reconnect...\n");
            // connect_and_setup();
            fmt::print("Disconnected from Qualisys server, exterminating...\n");
            break;
        }

        if (rtProtocol.ReceiveRTPacket(packetType, true) > 0) {
            if (packetType == CRTPacket::PacketData) {
                CRTPacket *rtPacket = rtProtocol.GetRTPacket();
                timestamp = rtPacket->GetTimeStamp();


                if (frameMode == "6D") {
                    // Reading 6DoF Bodies
                    for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); ++i) {
                        if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix)) {
                            const char *pTmpStr = rtProtocol.Get6DOFBodyName(i);
                            if (pTmpStr) {
                                // convert the ID to an integer
                                // @todo fix implementation to allow more than 1 digit for ID.
                                // @todo better processing for when frame is missed (value becomes nan)
                                int id = pTmpStr[0] - '0';
                                if (0 <= id && id < frames6D.size()) {
                                    // assign value to each frame
                                    // Qualisys data is in mm
                                    /** @todo if a frame wasn't even defined in QTM, it would not be populated with nan because this part won't run */
                                    if (std::isnan(fX) && !nan_if_missed)
                                        continue;
                                    frames6D[id](0, 3) = fX / 1000.;
                                    frames6D[id](1, 3) = fY / 1000.;
                                    frames6D[id](2, 3) = fZ / 1000.;
                                    for (int row = 0; row < 3; ++row) {
                                        for (int column = 0; column < 3; ++column) {
                                            // column-major order
                                            frames6D[id](row, column) = rotationMatrix[column * 3 + row];
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                else if (frameMode == "3D") {
                    // Reading 3D Motion Markers
                    for (unsigned int i = 0; i < rtPacket->Get3DMarkerCount(); ++i) {
                        if (rtPacket->Get3DMarker(i, fX, fY, fZ)) {
                            const char *pTmpStr = rtProtocol.Get3DLabelName(i);
                            if (pTmpStr) {
                                // ID looks like "1 - 4", depends on how the name is given.
                                // Let's assume that QTM always sends them in the right order.
                                if (i < frames3D.size()) {
                                    // assign value to each frame
                                    // Qualisys data is in mm
                                    if (std::isnan(fX) && !nan_if_missed)
                                        continue;
                                    frames3D[i](0) = fX / 1000.;
                                    frames3D[i](1) = fY / 1000.;
                                    frames3D[i](2) = fZ / 1000.;
                                }
                            }
                        }
                    }
                }
                else {
                    throw std::runtime_error("Failed to recognize frame type (3D or 6D).\n");
                }
                


                for (unsigned int i = 0; i < rtPacket->GetImageCameraCount(); ++i) {
                    for (int j = 0; j < cameraIDs.size(); j++){
                        // find camera 
                        if (rtPacket->GetImageCameraId(i) == cameraIDs[j]){
                            // read and decode the image for camera j
                            /** @todo this process could probably be made more efficient */
                            unsigned int w, h;
                            rtPacket->GetImageSize(i, w, h);
                            unsigned int image_size = rtPacket->GetImageSize(i);
                            // fmt::print("found image, id:{}\twidth:{}\theight:{}\tsize:{}\n", i, w, h, image_size);
                            assert(image_size < sizeof(data)/sizeof(data[0])); // if this fails, it means more memory must be allocated to data
                            rtPacket->GetImage(i, data, image_size);
                            rawImage = cv::Mat(1, image_size, CV_8SC1, (void*) data);
                            cv::imdecode(rawImage, cv::IMREAD_COLOR, &images[j]);
                        }
                    }
                }
            }
        }
    }
}


void QualisysClient::getData(std::vector<Eigen::Transform<double, 3, Eigen::Affine>> &frames,
                             unsigned long long int &timestamp) {
    std::lock_guard<std::mutex> lock(mtx);
    frames = this->frames6D;
    timestamp = this->timestamp;
}

void QualisysClient::getData(std::vector<Eigen::Vector3d> &frames,
                             unsigned long long int &timestamp) {
    std::lock_guard<std::mutex> lock(mtx);
    frames = this->frames3D;
    timestamp = this->timestamp;
}

void QualisysClient::getImage(int id, cv::Mat& image){
    assert(0 <= id && id < cameraIDs.size());
    image = images[id];
}

QualisysClient::~QualisysClient() {
    fmt::print("stopping QualisysClient\n");
    rtProtocol.Disconnect();
    motiontrack_thread.join();
    rtProtocol.StopCapture();
}

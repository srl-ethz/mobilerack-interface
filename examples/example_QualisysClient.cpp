//
// Created by yasu on 15/12/2020.
//

#include "mobilerack-interface/QualisysClient.h"

int main(){
    int num_frames = 2;
    std::vector<int> cameras = {9};
    QualisysClient qc{"192.168.0.101", num_frames, cameras};
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> frames;
    cv::Mat image;
    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    unsigned long long int timestamp;
    while (true) {
        qc.getData(frames, timestamp);
        fmt::print("timestamp: {}", timestamp);
        for (int j = 0; j < num_frames; ++j) {
            fmt::print("frame ID:\t{}\n{}\n", j, frames[j].matrix());
        }
        qc.getImage(0, image);
        if (image.rows > 0)
            cv::imshow("image", image);
        cv::waitKey(1); /** @todo find better way than to force sleep? */
            
        fmt::print("---------------\n");
        srl::sleep(0.5);
    }
}
//
// Created by yasu on 15/12/2020.
//

#include "mobilerack-interface/QualisysClient.h"

int main(){
    int num_frames = 2;
    QualisysClient qc{"192.168.0.0", num_frames, true};
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> frames;
    cv::Mat image1, image2;
    cv::namedWindow("image1", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("image2", CV_WINDOW_AUTOSIZE);
    unsigned long long int timestamp;
    while (true) {
        qc.getData(frames, timestamp);
        for (int j = 0; j < num_frames; ++j) {
            fmt::print("frame ID:\t{}\n{}\n", j, frames[j].matrix());
        }
        qc.getImage(image1, image2);
        if (image1.rows > 0)
            cv::imshow("image1", image1);
        if (image2.rows > 0)
            cv::imshow("image2", image2);
        cv::waitKey(1); /** @todo find better way than to force sleep? */
            
        fmt::print("---------------\n");
        sleep(0.5);
    }
}
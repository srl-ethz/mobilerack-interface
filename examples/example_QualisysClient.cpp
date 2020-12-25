//
// Created by yasu on 15/12/2020.
//

#include "mobilerack-interface/QualisysClient.h"

int main(){
    int num_frames = 2;
    QualisysClient qc{"192.168.0.0", 22222, num_frames};
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> frames;
    unsigned long long int timestamp;
    while (true) {
        qc.getData(frames, timestamp);
        for (int j = 0; j < num_frames; ++j) {
            fmt::print("frame ID:\t{}\n{}\n", j, frames[j].matrix());
        }
        fmt::print("---------------\n");
        sleep(0.5);
    }
}
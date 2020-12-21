//
// Created by yasu on 15/12/2020.
//

#include "QualisysClient.h"

int main(){
    int num_frames = 2;
    QualisysClient qc{num_frames};
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> frames;
    while (true) {
        qc.getData(frames);
        for (int j = 0; j < num_frames; ++j) {
            fmt::print("frame ID:\t{}\n", j);
            std::cout << frames[j].matrix() << std::endl;
        }
        fmt::print("---------------\n");
        sleep(1);
    }
}
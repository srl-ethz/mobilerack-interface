#include "mobilerack-interface/QualisysClient.h"


#include <iostream>
#include <fstream>

int main(){
    int number_of_markers = 8;
    std::vector<int> cameras = {0,1,2,3,4,5,6,7};
    QualisysClient qc{number_of_markers, cameras, "3D"};

    srl::sleep(1); // hacky way to wait until data from QTM is received
    std::vector<std::vector<std::vector<Eigen::Vector3d>>> captured_markers;
    std::vector<std::vector<Eigen::Vector3d>> frame_list;
    std::vector<Eigen::Vector3d> frames;
    unsigned long long int timestamp;
    unsigned long long int curr_stamp;


    // Sync timestamp
    qc.getData(frames, timestamp);

    const int timesteps = 200;

    curr_stamp = timestamp;

    for (int t = 0; t < timesteps; t++) {
        while (timestamp == curr_stamp) {
            qc.getData(frames, timestamp);
        }
        frame_list.push_back(frames);
        curr_stamp = timestamp;
    }

    captured_markers.push_back(frame_list);
    srl::sleep(1);


    // Start writing to the file
    std::ofstream dataFile("captured_markers.txt");
    dataFile << "STORED DATA";

    for (int t = 0; t < timesteps; t++) {
        dataFile << "\n\nStep" << t << std::endl;
        for (int i = 0; i < number_of_markers; i++) {
            for (int j = 0; j < 3; j++) {
                dataFile << captured_markers[t][i][j] << "\t";
            }
            dataFile << std::endl;
        }
    }


    dataFile.close();

    return 1;
}

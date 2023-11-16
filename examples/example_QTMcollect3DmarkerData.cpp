#include "mobilerack-interface/QualisysClient.h"
#include "mobilerack-interface/ValveController.h"


#include <iostream>
#include <fstream>

int main(){
    const int num_pressures = 35;
    const int num_actuations = 3;
    const int max_pressure = 400;
    int valve_id;
    std::vector<int> valves = {0, 1, 2};
    // ValveController vc{"192.168.0.100", valves, max_pressure};


    int number_of_markers = 8;
    std::vector<int> cameras = {0,1,2,3,4,5,6,7, 8};
    QualisysClient qc{number_of_markers, cameras, "3DNoLabels"};

    srl::sleep(1); // hacky way to wait until data from QTM is received
    std::vector<std::vector<std::vector<Eigen::Vector3d>>> captured_markers;
    std::vector<std::vector<Eigen::Vector3d>> frame_list;
    std::vector<Eigen::Vector3d> frames;
    unsigned long long int timestamp;
    unsigned long long int curr_stamp;


    // Sync timestamp
    qc.getData(frames, timestamp);
    fmt::print("syncing ValveController log time to that of QTM: {}\n", timestamp/1000);
    // vc.syncTimeStamp(timestamp/1000); // sync timestamp to that of QTM


    const int timesteps = 200;

    for (int k = 0; k < num_pressures; k++) {
        curr_stamp = timestamp;
        std::cout << "k: " << k << " timestamp: " << timestamp << std::endl;
        for (int t = 0; t < timesteps; t++) {
            while (timestamp == curr_stamp) {
                qc.getData(frames, timestamp);
            }
            frame_list.push_back(frames);
            curr_stamp = timestamp;


            // for (int i = 0; i < valves.size(); i++) {
            //     vc.setSinglePressure(i, pressures[k][i]);
            // }
        }

        captured_markers.push_back(frame_list);

        // // Reset to 0
        // for (int i = 0; i < valves.size(); i++) {
        //     vc.setSinglePressure(i, 0);
        // }
        srl::sleep(1);
    }


    // Start writing to the file
    std::ofstream dataFile("captured_markers.txt");
    dataFile << "STORED DATA";

    for (int k = 0; k < num_pressures; k++) {
        for (int t = 0; t < timesteps; t++) {
            std::cout << "k: " << k << " timestep: " << t << std::endl;
            dataFile << "\n\nStep" << t << std::endl;
            for (int i = 0; i < number_of_markers; i++) {
                for (int j = 0; j < 3; j++) {
                    dataFile << captured_markers[k][t][i][j] << "\t";
                }
                dataFile << std::endl;
            }
        }
    }

    dataFile.close();

    return 1;
}

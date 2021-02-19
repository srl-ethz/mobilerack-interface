#include "mobilerack-interface/QualisysClient.h"
#include "mobilerack-interface/ValveController.h"

int main(){
    const int pressure = 300;
    int valve_id;
    std::vector<int> map = {12, 13, 14, 15};

    ValveController vc{"192.168.0.100", map, 400};
    QualisysClient qc{"192.168.0.101", 0};

    srl::sleep(1); // hacky way to wait until data from QTM is received

    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> frames_; // will not be read
    unsigned long long int timestamp;
    qc.getData(frames_, timestamp);
    fmt::print("time: {}", timestamp);
    fmt::print("time milli: {}", timestamp/1000);

    vc.syncTimeStamp(timestamp/1000); // sync timestamp to that of QTM

    for (int i = 0; i < map.size(); i++) {
        valve_id = map[i];
        std::cout << "actuator ID:\t" << i << "\tvalve ID:\t" << valve_id << "\tpressure\t" << pressure << std::endl;
        vc.setSinglePressure(i, pressure);
        srl::sleep(1);
        vc.setSinglePressure(i, 0);
        srl::sleep(1);
    }
    return 1;
}


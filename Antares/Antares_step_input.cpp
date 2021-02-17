#include "mobilerack-interface/ValveController.h"
/**
 * @file example_ValveController.cpp
 * @brief This program shows an example usage of the ValveController. Actuates through each valve defined in map
 */

int main() {
    const int pressure = 200;
    const int total_time = 15;
    int valve_id;
    // 0--> value 14 --> Fish left side
    // 1--> value 15 --> Fish right side
    double freq = 4;
    double cycle_time = 1/freq;
    int cycle_count = total_time/cycle_time;
    std::vector<int> map = {14, 15};
    ValveController vc{"192.168.0.100", map, 400};
    for (int j = 0; j < cycle_count; j++){
        vc.setSinglePressure(0, pressure);
        vc.setSinglePressure(1, 0);
        srl::sleep(cycle_time);
        vc.setSinglePressure(0, 0);
        vc.setSinglePressure(1, pressure);
        srl::sleep(cycle_time);


        // for (int i = 0; i < map.size(); i++) {
        //     valve_id = map[i];
        //     std::cout << "actuator ID:\t" << i << "\tvalve ID:\t" << valve_id << "\tpressure\t" << pressure << std::endl;
        //     vc.setSinglePressure(i, pressure);
        //     sleep(1);
        //     vc.setSinglePressure(i, 0);
        //     sleep(1);
        // }
    }
    
    vc.disconnect();
    return 1;
}
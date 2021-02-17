#include "mobilerack-interface/ValveController.h"
/**
 * @file example_ValveController.cpp
 * @brief This program shows an example usage of the ValveController. Actuates through each valve defined in map
 */

int main() {
    const int pressure = 200;
    int valve_id;
    std::vector<int> map = {14, 15};
    ValveController vc{"192.168.0.100", map, 400};
    for (int j = 0; j < 5; j++){
        for (int i = 0; i < map.size(); i++) {
            valve_id = map[i];
            std::cout << "actuator ID:\t" << i << "\tvalve ID:\t" << valve_id << "\tpressure\t" << pressure << std::endl;
            vc.setSinglePressure(i, pressure);
            srl::sleep(1);
            vc.setSinglePressure(i, 0);
            srl::sleep(1);
        }
    }
    
    vc.disconnect();
    return 1;
}
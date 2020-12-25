#include "ValveController.h"
#include <chrono>
#include <iostream>
#include <thread>
/**
 * @file example_ValveController.cpp
 * @brief This program shows an example usage of the ValveController. Actuates through each valve defined in st_params::valve::map
 */

int main() {
    const int pressure = 300;
    int valve_id;
    std::vector<int> map = {0, 1, 2, 3};
    ValveController vc{"192.168.0.0", map, 400};
    for (int i = 0; i < map.size(); i++) {
        valve_id = map[i];
        std::cout << "actuator ID:\t" << i << "\tvalve ID:\t" << valve_id << "\tpressure\t" << pressure << std::endl;
        vc.setSinglePressure(i, pressure);
        sleep(1);
        vc.setSinglePressure(i, 0);
        sleep(1);
    }
    vc.disconnect();
    return 1;
}
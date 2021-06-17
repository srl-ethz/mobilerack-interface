#include "mobilerack-interface/ValveController.h"
/**
 * @file example_ValveController.cpp
 * @brief This program shows an example usage of the ValveController. Actuates through each valve defined in map
 */

int main() {
    const int pressure = 300;  //pressure variable value in mBar, can define multiple different ones
    int valve_id;
    std::vector<int> map = {0, 1, 2, 3, 4};         //select valves 0, 1, 2, 3, ... 15
    ValveController vc{"192.168.0.100", map, 2000}; //last parameter sets max. pressure in mBar
    
    
    vc.setSinglePressure(1, 1800);      //sets pressure for valve i (i-th entry of the selected valves array), second parameter is pressure in mBar
    srl::sleep(3);                      //wait 1sec
    vc.setSinglePressure(1, 0);         //set pressure back to 0
    srl::sleep(1);
    
    
    
    /*
    for (int i = 0; i < map.size(); i++) {
        valve_id = map[i];
        std::cout << "actuator ID:\t" << i << "\tvalve ID:\t" << valve_id << "\tpressure\t" << pressure << std::endl;
        vc.setSinglePressure(i, pressure);  //sets pressure for valve i?
        srl::sleep(2);                      //wait 2sec? 
        vc.setSinglePressure(i, 0);         //set pressure back to 0
        srl::sleep(1);
    }
    return 1;

    */
}

#include "mobilerack-interface/ValveController.h"
/**
 * @file example_ValveController.cpp
 * @brief This program shows an example usage of the ValveController. Actuates through each valve defined in map
 */

int main() {
    //const int pressure = 500;  //pressure variable value in mBar, can define multiple different ones
    int valve_id;
    std::vector<int> map = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};         //select valves 0, 1, 2, 3, ... 15
    ValveController vc{"192.168.0.100", map, 2000}; //last parameter sets max. pressure in mBar
    



// hand prototype 3 v1
// Fingertip forces load-cell

    //set metacarpal joint angle
    //const int angle = 90;
    //translate angle
    //const int pressure = angle * 1000 / 90;  // 90 degrees = 2bars

    double duration = 5;
    double timestep = 0.01;
    double p;
    double pt;

    //set lead pressure in mbar
    int pressure = 2000; //1800 easy





////////////////////////////////////////////////////////////////


    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);    
        
        //middle flexion
        vc.setSinglePressure(0, 0.95*p);  //middle middle finger       
        vc.setSinglePressure(1, 0.85*p);  //bottom middle finger      
        vc.setSinglePressure(2, p);  //top middle finger  

        srl::sleep(timestep);
    }

    srl::sleep(2);

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);       
        
        //middle relax
        vc.setSinglePressure(0, 0.95*(pressure-p) );  //middle middle finger       
        vc.setSinglePressure(1, 0.85*(pressure-p) );  //bottom middle finger      
        vc.setSinglePressure(2, pressure-p);  //top middle finger  

        srl::sleep(timestep);
        }

    srl::sleep(2);





















    //End / Relax all
    vc.setSinglePressure(0, 0);
    vc.setSinglePressure(1, 0);         
    vc.setSinglePressure(2, 0);
    vc.setSinglePressure(3, 0);
    vc.setSinglePressure(4, 0);         
    vc.setSinglePressure(5, 0);
    vc.setSinglePressure(6, 0);
    vc.setSinglePressure(7, 0);         
    vc.setSinglePressure(8, 0);
    vc.setSinglePressure(9, 0);
    vc.setSinglePressure(10, 0);         
    vc.setSinglePressure(11, 0);
    vc.setSinglePressure(12, 0);
    vc.setSinglePressure(13, 0);         
    vc.setSinglePressure(14, 0);
    vc.setSinglePressure(15, 0);




}

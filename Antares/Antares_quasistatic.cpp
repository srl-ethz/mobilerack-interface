#include "mobilerack-interface/ValveController.h"
#include "mobilerack-interface/SerialInterface.h"
/**
 * @file example_ValveController.cpp
 * @brief This program shows an example usage of the ValveController. Actuates through each valve defined in map
 */

int main() {
    SerialInterface si = SerialInterface("/dev/ttyACM0", 9600);

    int valve_id;
    // 0--> value 14 --> Fish right side --> left in picture
    // 1--> value 15 --> Fish left side --> right in picture
    double wait_time = 5;

    std::vector<int> map = {15, 14};
    ValveController vc{"192.168.0.100", map, 400};
    si.sendData(true);//start of experiment, light LED

    vc.setSinglePressure(1, 50);//actuate value 14 --> expand left chamber bend to right
    vc.setSinglePressure(0, 0);
    srl::sleep(wait_time);// wait to stablize

    vc.setSinglePressure(1, 100);//actuate value 14 --> expand left chamber bend to right
    vc.setSinglePressure(0, 0);
    srl::sleep(wait_time);// wait to stablize

    vc.setSinglePressure(1, 150);//actuate value 14 --> expand left chamber bend to right
    vc.setSinglePressure(0, 0);
    srl::sleep(wait_time);// wait to stablize

    vc.setSinglePressure(1, 200);//actuate value 14 --> expand left chamber bend to right
    vc.setSinglePressure(0, 0);
    srl::sleep(wait_time);// wait to stablize

    vc.setSinglePressure(1, 250);//actuate value 14 --> expand left chamber bend to right
    vc.setSinglePressure(0, 0);
    srl::sleep(wait_time);// wait to stablize

    vc.setSinglePressure(1, 275);//actuate value 14 --> expand left chamber bend to right
    vc.setSinglePressure(0, 0);
    srl::sleep(wait_time);// wait to stablize

    vc.setSinglePressure(1, 0);//release pressure and start next direction
    vc.setSinglePressure(0, 0);
    srl::sleep(wait_time);// wait to stablize

    vc.setSinglePressure(1, 0);//actuate value 15 --> expand right chamber bend to left
    vc.setSinglePressure(0, 50);
    srl::sleep(wait_time);// wait to stablize

    vc.setSinglePressure(1, 0);//actuate value 15 --> expand right chamber bend to left
    vc.setSinglePressure(0, 100);
    srl::sleep(wait_time);// wait to stablize

    vc.setSinglePressure(1, 0);//actuate value 15 --> expand right chamber bend to left
    vc.setSinglePressure(0, 150);
    srl::sleep(wait_time);// wait to stablize

    vc.setSinglePressure(1, 0);//actuate value 15 --> expand right chamber bend to left
    vc.setSinglePressure(0, 200);
    srl::sleep(wait_time);// wait to stablize

    vc.setSinglePressure(1, 0);//actuate value 15 --> expand right chamber bend to left
    vc.setSinglePressure(0, 250);
    srl::sleep(wait_time);// wait to stablize

    vc.setSinglePressure(1, 0);//actuate value 15 --> expand right chamber bend to left
    vc.setSinglePressure(0, 275);
    srl::sleep(wait_time);// wait to stablize

    vc.setSinglePressure(1, 0);//realease pressure
    vc.setSinglePressure(0, 0);
    srl::sleep(wait_time);// wait to stablize
    
    si.sendData(false);//end of experiment, turn off LED
    vc.disconnect();
    return 1;
}
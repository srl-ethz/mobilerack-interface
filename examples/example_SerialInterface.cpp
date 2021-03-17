//
// Created by yasu on 02/01/2021.
//

#include "mobilerack-interface/SerialInterface.h"

/*
use together with the included Arduino sketch. Reads data sent from Arduino, and sends true->false->true->false to Arduino to toggle LED.
*/

int main() {
    SerialInterface si = SerialInterface("/dev/cu.usbmodem14201", 38400);
    srl::Rate r{10};
    std::vector<float> data;
    bool flag = true;
    for (int i = 0; i < 100; i++)
    {
        si.getData(data);
        si.sendData(flag);
        flag = !flag;
        fmt::print("data: {}\n", data);
        r.sleep();
    }
    return 1;
}
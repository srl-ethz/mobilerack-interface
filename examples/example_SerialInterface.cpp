//
// Created by yasu on 02/01/2021.
//

#include "mobilerack-interface/SerialInterface.h"

/*
use together with the included Arduino sketch.
*/

int main() {
    SerialInterface si = SerialInterface("/dev/cu.usbmodem14201", 38400);
    srl::Rate r{10};
    std::vector<float> data;
    for (int i = 0; i < 100; i++)
    {
        si.getData(data);
        fmt::print("data: {}\n", data);
        r.sleep();
    }
    return 1;
}
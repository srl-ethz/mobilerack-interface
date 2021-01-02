//
// Created by yasu on 02/01/2021.
//

#include "mobilerack-interface/SerialInterface.h"

int main() {
    SerialInterface si = SerialInterface("/dev/ttyUSB0");
    return 1;
}
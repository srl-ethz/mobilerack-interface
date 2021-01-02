//
// Created by yasu on 02/01/2021.
//
#pragma once

#include "common.h"
#include <mutex>
#include "libserial/SerialStream.h"
#include "libserial/SerialPort.h"
#include "fmt/core.h"
#include "fmt/ostream.h"

/** @brief wrapper for libserial library to make it easier to read out serial data from Arduino.
 * WSL2 doesn't support USB devices, so can only be run on Linux (and maybe macOS).
 *
 * might be better to use external library for reading from serial data- some candidates are
 * - https://github.com/araffin/cpp-arduino-serial
 * - https://github.com/manashmandal/SerialPort
 * - https://github.com/imabot2/serialib
 * - https://github.com/wjwwood/serial
 */
class SerialInterface{
public:
    SerialInterface(std::string port);
    /** @brief get latest data from the serial port.
     * @param data
     */
    void getData(std::vector<float>& data);

private:
    LibSerial::SerialStream serialStream;
    std::thread serial_thread;
    std::mutex mtx;
    void serial_loop();
    /** @brief data received from arduino is stored here */
    float data[1];

    static const int max_buffer_size = 200;
    /** @brief this buffer will continuously be filled with new data bytes from Arduino. discards old data when it overflows. */
    char buffer[max_buffer_size];
    int current_buffer_size = 0;

    /** @brief parse the latest data found in the buffer and cleanse the buffer up till that point. */
    void parse_latest_data();
};

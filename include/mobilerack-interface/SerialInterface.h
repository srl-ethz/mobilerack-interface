//
// Created by yasu on 02/01/2021.
//
#pragma once

#include "common.h"
#include <mutex>
#include <libserialport.h>
#include "fmt/core.h"
#include "fmt/ostream.h"
#include "fmt/ranges.h"

/** @brief wrapper for libserial library to make it easier to read out serial data from Arduino.
 * WSL2 doesn't support USB devices, so can only be run on Linux and macOS.
 * data byte array format is:
 * 1. [0 - 23]: 6 float values (each is 4 bytes)
 * 1. [24]: 13 (carriage return character, '\r')
 * 1. [25]: 10 (newline character, '\\n')
 * 
 * last 2 bytes are taken from behavior of [println()](https://www.arduino.cc/reference/en/language/functions/communication/serial/println/) on Arduino. 
 * 
 * @todo possibly messes up if footer 2 bytes appear with float encoded segment, must be made more robust
 * @todo support more formats for data, not just 6 floats
 * @todo this is a rather rough implementation, so always make sure the data seems appropriate.
 *
 * might be better to use external library for reading from serial data- some candidates are
 * - https://github.com/araffin/cpp-arduino-serial
 * - https://github.com/manashmandal/SerialPort
 * - https://github.com/imabot2/serialib
 * - https://github.com/wjwwood/serial
 * 
 * cf: https://gist.github.com/Nixes/78e401234e66aa131547d7b78135271c
 */
class SerialInterface{
public:
    /**
     * @brief Construct a new Serial Interface object
     * 
     * @param portname name of serial port, can check with Arduino IDE etc.
     * @param baud_rate 38400 recommended
     */
    SerialInterface(std::string portname, int baud_rate);
    ~SerialInterface();
    /** @brief get latest data from the serial port.
     */
    void getData(std::vector<float>& data);

private:
    struct sp_port *port;
    std::thread serial_thread;
    std::mutex mtx;
    void serial_loop();
    /** @brief data received from arduino is stored here */
    float data[6];

    static const int max_buffer_size = 10000;
    /** @brief this buffer will continuously be filled with new data bytes from Arduino. discards old data when it overflows. */
    char buffer[max_buffer_size];
    /** @brief i.e. buffer[current_buffer_size-1] is the last data in buffer */
    int current_buffer_size = 0;

    /** @brief parse the latest data found in the buffer and delete the buffer up till that point. */
    void parse_latest_data();
};

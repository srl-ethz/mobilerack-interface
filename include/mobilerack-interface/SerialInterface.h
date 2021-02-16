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
 * Currently only supports 6 float values.
 * 
 * send serial data from arduino as ascii string beginning with 'a', values separated by comma, and end with a newline character, with no spaces, i.e. "a3.14,1,2,3,4,5\n"
 * 
 * Of course it's more efficient to send the actual data rather than as ASCII string, but then it's tricky how to recoginize where data starts & ends (cf: https://forum.arduino.cc/index.php?topic=628401.0 )
 * 
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
    /** @brief if true, send "H" over serial to Arduino, if false, send "L". `sudo` may be required for writing to the serial port.
     * @todo support more data and not just a bool value
     */
    void sendData(bool flag);

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

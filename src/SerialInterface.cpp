//
// Created by yasu on 02/01/2021.
//

#include "mobilerack-interface/SerialInterface.h"

SerialInterface::SerialInterface(std::string port) {
    fmt::print("opening port {}\n", port);
    serialStream.Open(port);
    serialStream.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_thread = std::thread(&SerialInterface::serial_loop, this);
}

void SerialInterface::getData(std::vector<float>& data){
    data.resize(sizeof(this->data)/sizeof(this->data[0]));
    for (int i = 0; i < data.size(); ++i) {
        data[i] = this->data[i];
    }
}

void SerialInterface::serial_loop(){
    const int read_size = 10;
    while(true){
        while (current_buffer_size + read_size > max_buffer_size){
            // clear the buffer until there's enough space to read next batch of data
            memmove(buffer, &buffer[read_size], max_buffer_size-read_size);
            current_buffer_size -= read_size;
        }
        serialStream.read(&buffer[current_buffer_size], read_size);
        current_buffer_size += read_size;
        parse_latest_data();
    }
}

void SerialInterface::parse_latest_data(){
    // first, find the last newline character in buffer
    int last_newline_index = -1;
    for (int i = 0; i < current_buffer_size; ++i) {
        if (buffer[i] == '\n')
            last_newline_index = i;
    }
    int data_begin_index = last_newline_index - sizeof(data);
    if (last_newline_index < 0 || data_begin_index < 0)
        return; // did not find newline, or not enough data received yet

    // read out the data
    memcpy(&data[0], &buffer[data_begin_index], sizeof(data[0]));

    // remove the read part from the buffer
    memmove(buffer, &buffer[last_newline_index+1], max_buffer_size - (last_newline_index+1));
    current_buffer_size -= last_newline_index+1;
}
//
// Created by yasu on 02/01/2021.
//

#include "mobilerack-interface/SerialInterface.h"

SerialInterface::SerialInterface(std::string portname) {
    fmt::print("opening port {}\n", portname);
    sp_return error = sp_get_port_by_name(portname.c_str(),&port);
    if (error == SP_OK) {
        error = sp_open(port,SP_MODE_READ);
        if (error == SP_OK) {
            fmt::print("succeeded opening serial device {}\n", portname);
            sp_set_baudrate(port,115200);
        } else 
        fmt::print("Error opening serial device: {}\n", portname);
    } 
    else 
        fmt::print("Error finding serial device: {}\n", portname);
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
          while(true) {
        sleep(0.5); // can do something else in mean time
        int bytes_waiting = sp_input_waiting(port);
        if (bytes_waiting > 0) {
          printf("Bytes waiting %i\n", bytes_waiting);
          char byte_buff[512];
          int byte_num = 0;
          byte_num = sp_nonblocking_read(port,byte_buff,512);
          // parse_serial(byte_buff,byte_num);
        }
        fflush(stdout);
      }

      sp_close(port);
    while(true){
        // while (serialStream.IsDataAvailable()){
        //     // clear the buffer until there's enough space to read next batch of data
        //     current_buffer_size -= read_size;
        // }
        std::string data_string;
        // serialStream.ReadLine(data_string);
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
//
// Created by yasu on 02/01/2021.
//

#include "mobilerack-interface/SerialInterface.h"

SerialInterface::SerialInterface(std::string portname, int baud_rate) {
    fmt::print("opening port {}\n", portname);
    sp_return error = sp_get_port_by_name(portname.c_str(),&port);
    if (error == SP_OK) {
        error = sp_open(port,SP_MODE_READ);
        if (error == SP_OK) {
            fmt::print("succeeded opening serial device {}\n", portname);
            sp_set_baudrate(port,baud_rate);
            fmt::print("set baud rate to {}\n", baud_rate);
        } else 
        fmt::print("Error opening serial device: {}\n", portname);
    }
    else 
        fmt::print("Error finding serial device: {}\n", portname);
    serial_thread = std::thread(&SerialInterface::serial_loop, this);
}

SerialInterface::~SerialInterface(){
    fmt::print("closing serial device\n");
    sp_close(port);
}

void SerialInterface::getData(std::vector<float>& data){
    std::lock_guard<std::mutex> lock(mtx);
    data.resize(sizeof(this->data)/sizeof(this->data[0]));
    for (int i = 0; i < data.size(); ++i) {
        data[i] = this->data[i];
    }
}

void SerialInterface::serial_loop(){
    const int read_size = 1000; /** @brief read up to this many bytes on a single read */
    int bytes_read; /** @brief how many bytes were actually read */
    int bytes_to_delete; /** @brief how many bytes to delete from buffer (to prevent overflowing) */
    while (true)
    {
        if (current_buffer_size + read_size > max_buffer_size)
        {
            // clear the buffer to make enough space to read next batch of data
            bytes_to_delete = current_buffer_size + read_size - max_buffer_size;
            memmove(buffer, &buffer[bytes_to_delete], max_buffer_size - bytes_to_delete);
            current_buffer_size -= bytes_to_delete;
            fmt::print("deleted {} bytes to make space (this shouldn't happen that often, check code)\n", bytes_to_delete);
        }

        // read the incoming data
        bytes_read = sp_blocking_read_next(port, &buffer[current_buffer_size], read_size, 0);
        if (bytes_read < 0)
            continue;
        current_buffer_size += bytes_read;

        if (current_buffer_size > sizeof(data))
            parse_latest_data();
    }
}

void SerialInterface::parse_latest_data(){
    // first, find the last newline character in buffer
    int last_newline_index = -1;
    for (int i = 0; i < current_buffer_size - 1; ++i) {
        if (buffer[i] == 13 && buffer[i+1] == 10)
            last_newline_index = i;
    }
    if (last_newline_index < 0)
        return; // did not find newline
    int data_begin_index = last_newline_index - sizeof(data);
    // read out the data
    {
        std::lock_guard<std::mutex> lock(mtx);
        for (int i = 0; i < sizeof(data)/sizeof(data[0]); i++)
        {
            memcpy(&data[i], &buffer[data_begin_index + sizeof(data[0])*i], sizeof(data[0]));
        }
    }

    // remove the read part from the buffer
    memmove(buffer, &buffer[last_newline_index+2], max_buffer_size - (last_newline_index+2));
    current_buffer_size -= last_newline_index+2;
}
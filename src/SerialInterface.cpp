//
// Created by yasu on 02/01/2021.
//

#include "mobilerack-interface/SerialInterface.h"

SerialInterface::SerialInterface(std::string portname, int baud_rate) {
    fmt::print("opening port {}\n", portname);
    sp_return error = sp_get_port_by_name(portname.c_str(),&port);
    if (error == SP_OK) {
        error = sp_open(port,SP_MODE_READ_WRITE);
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

void SerialInterface::sendData(bool flag){
    char flag_byte = 'L';
    if (flag)
        flag_byte = 'H';
    sp_blocking_write(port, &flag_byte, 1, 0);
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
    for (int i = 0; i < current_buffer_size; ++i) {
        if (buffer[i] == '\n')
            last_newline_index = i;
    }
    if (last_newline_index < 0)
        return; // did not find newline
    // find beginning of data
    int data_begin_index = 0;
    for (int i = last_newline_index-1; 0 <= i; i-=1)
    {
        if (buffer[i] == '\n'){
            // this is the newline from the previous data which is still in buffer.
            data_begin_index = i+1;
            break;
        }
    }
    
    int float_head_index[6]; // stores the index of where each float value starts
    float_head_index[0] = data_begin_index;
    int count = 1;
    // find all the commas, which separate the float values
    for (int i = float_head_index[0] ; i < last_newline_index-1; i++)
    {
        if (buffer[i] == ',')
        {
            float_head_index[count] = i+1;
            count++;
        }
    }
    if (count == 6){
        for (int i = 0; i < 6; i++)
        {
            std::lock_guard<std::mutex> lock(mtx);
            data[i] = std::atof(&buffer[float_head_index[i]]);
        }
    }
    else
        fmt::print("WARNING: detected line not conforming to SerialInterface protocol: {}\n", std::string(&buffer[data_begin_index], last_newline_index - data_begin_index + 1));

    // remove the read part from the buffer
    memmove(buffer, &buffer[last_newline_index+1], max_buffer_size - (last_newline_index+1));
    current_buffer_size -= last_newline_index+1;
}
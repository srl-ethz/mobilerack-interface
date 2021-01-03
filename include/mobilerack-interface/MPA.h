#pragma once

#include <cstdint>
#include <vector>

#include "modbus.h"
#include "modbus-tcp.h"

/**
 * @brief low-level control for the Festo valve array
 */
class MPA {
public:
    explicit MPA(const char *node, const char *service);

    ~MPA();

    MPA &operator=(const MPA &) = delete;

    MPA(const MPA &) = delete;

    MPA() = default;

    bool connect();

    bool disconnect();

    int get_single_pressure(int index);

    void set_single_pressure(int index, int pressure);

    void get_all_pressures(std::vector<int> *output);

    void set_all_pressures(const std::vector<int> &pressures);

private:
    void ensure_connection() const;

    std::vector<uint16_t> input_buffer_;
    std::vector<uint16_t> output_buffer_;

    bool connected_;
    modbus_t *ctx_;
};

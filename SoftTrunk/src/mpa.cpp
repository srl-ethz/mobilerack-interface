#include "modbus-tcp.h"
#include "mpa.h"
#include <stdexcept>


namespace {
	const int address_input_start = 45392;
	const int address_output_start = 40001;
	const int cpx_input_offset = 3;
	const int cpx_output_offset = 2;
	const int num_valves = 16;
}

MPA::MPA(const char* node, const char* service) {
	// Not connected.
	connected_ = false;

	// Resize buffer space.
	// Input buffer for each valve is of the format (actual, setpoint, diagnostic).
	input_buffer_.resize(3 * num_valves);
	output_buffer_.resize(num_valves);

	// Create Modbus context.
	ctx_ = modbus_new_tcp_pi(node, service);
}

MPA::~MPA() {
	// Close if not yet closed.
	if (connected_) {
		disconnect();
	}

	// Deallocate Modbus context.
	modbus_free(ctx_);
}

bool MPA::connect() {
	if (modbus_connect(ctx_) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx_);
		connected_ = false;
		return false;
	}
	connected_ = true;
	return true;
}

bool MPA::disconnect() {
	if (connected_) {
		modbus_close(ctx_);
		connected_ = false;
		return true;
	}

	return false;
}

void MPA::ensure_connection() const {
	if (!connected_) {
		throw std::runtime_error("Operation requires a connection.");
	}
}

int MPA::get_single_pressure(const int index) {
	ensure_connection();
	const auto dest = &input_buffer_[index];
	const auto addr = address_input_start + cpx_input_offset + 3 * index;

	if (modbus_read_registers(ctx_, addr, 1, dest) == -1) {
		throw std::runtime_error("Failed to read VPPM register.");
	}

	return *dest;
}

void MPA::set_single_pressure(const int index, const int pressure) {
	ensure_connection();
	const auto addr = address_output_start + cpx_output_offset + index;

	if (modbus_write_register(ctx_, addr, pressure) == -1) {
		throw std::runtime_error("Failed to write VPPM register.");
	}
}

void MPA::get_all_pressures(std::vector<int>* output) {
	ensure_connection();
	const auto dest = &input_buffer_[0];
	const auto addr = address_input_start + cpx_input_offset;

	if (modbus_read_registers(ctx_, addr, num_valves * 3, dest) == -1) {
		throw std::runtime_error("Failed to read VPPM registers.");
	}

	// Only read the actual value and ignore setpoint/diagonstic.
	for (auto i = 0; i < num_valves; i++) {
		output->at(i) = input_buffer_[i * 3];
	}
}

void MPA::set_all_pressures(const std::vector<int>& pressures) {
	ensure_connection();
	const auto data = &output_buffer_[0];
	const auto addr = address_output_start + cpx_output_offset;

	for (auto i = 0; i < num_valves; i++) {
		output_buffer_[i] = pressures.at(i);
	}

	if (modbus_write_registers(ctx_, addr, num_valves, data) == -1) {
		throw std::runtime_error("Failed to write VPPM registers.");
	}
}

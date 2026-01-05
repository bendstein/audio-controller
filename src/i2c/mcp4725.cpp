//
// Created by bendstein on 12/28/2025.
//
#include "i2c/mcp4725.h"

bool mcp4725::dac::ping() const
{
    VERBOSE(get_log_key(), "Ping");

    return try_general_call(general_call::wake_up);
}

bool mcp4725::dac::try_write_value(const power_down_mode power_mode, const uint16_t value) const
{
    VERBOSE(get_log_key(), std::format("Write value {}, power mode {}", value, static_cast<uint8_t>(power_mode)));

    const uint16_t write_buffer_value = (value & 0x0FFF)
        | (static_cast<uint8_t>(power_mode) << 12)
        | (static_cast<uint8_t>(command_type::fast_mode) << 14);

    const uint8_t write_buffer[2] = {
        static_cast<uint8_t>((write_buffer_value & 0xFF00) >> 8),
        static_cast<uint8_t>(write_buffer_value & 0xFF)
    };

    VERBOSE(get_log_key(), std::format("Start write values {}, {}", write_buffer[0], write_buffer[1]));

    const auto write_result = i2c_master_transmit(
        handle,
        write_buffer, sizeof(write_buffer),
        timeout_ms
    );

    VERBOSE(get_log_key(), std::format("Write result {}", write_result));

    return write_result == ESP_OK;
}

bool mcp4725::dac::try_write_value(const uint16_t value) const
{
    return try_write_value(power_down_mode::normal, value);
}

bool mcp4725::dac::try_general_call(const general_call command) const
{
    VERBOSE(get_log_key(), std::format("General Call {}", static_cast<uint8_t>(command)));

    const uint8_t write_buffer[2] = {
        0,
        static_cast<uint8_t>(command)
    };

    VERBOSE(get_log_key(), std::format("Start write values {}, {}", write_buffer[0], write_buffer[1]));

    const auto write_result = i2c_master_transmit(
        handle,
        write_buffer, sizeof(write_buffer),
        timeout_ms
    );

    VERBOSE(get_log_key(), std::format("Write result {}", write_result));

    return write_result == ESP_OK;
}
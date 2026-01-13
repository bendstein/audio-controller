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
    try
    {
        assert(handle != nullptr);
        VERBOSE(get_log_key(), std::format("Write value 0x{:04X}, power mode 0x{:02X}.", value, static_cast<uint8_t>(power_mode)));

        const uint16_t write_buffer_value = (value & 0x0FFF)
            | (static_cast<uint8_t>(power_mode) << 12)
            | (static_cast<uint8_t>(command_type::fast_mode) << 14);

        const uint8_t write_buffer[2] = {
            static_cast<uint8_t>((write_buffer_value & 0xFF00) >> 8),
            static_cast<uint8_t>(write_buffer_value & 0xFF)
        };

        VERBOSE(get_log_key(), std::format("Start write values 0x{:02X}, 0x{:02X}", write_buffer[0], write_buffer[1]));

        const auto write_result = i2c_master_transmit(
            handle,
            write_buffer, sizeof(write_buffer),
            timeout_ms
        );

        VERBOSE(get_log_key(), std::format("Write result 0x{:02X}", write_result));

        if (write_result != ESP_OK)
        {
            loge(get_log_key(), std::format("Failed to write values 0x{:02X}, 0x{:02X}. [0x{:04X}] {}",
                write_buffer[0], write_buffer[1],
                write_result,
                esp_err_to_name(write_result)));
        }

        return write_result == ESP_OK;
    }
    catch (std::exception& e)
    {
        loge(get_log_key(), std::format("An exception occurred while writing value 0x{:04X}, power mode 0x{:02X}: {}",
            value, static_cast<uint8_t>(power_mode),
            e.what()));

        return false;
    }
}

bool mcp4725::dac::try_write_value(const uint16_t value) const
{
    return try_write_value(power_down_mode::normal, value);
}

bool mcp4725::dac::try_general_call(const general_call command) const
{
    try
    {
        assert(handle != nullptr);
        VERBOSE(get_log_key(), std::format("General Call 0x{:02X}", static_cast<uint8_t>(command)));

        const uint8_t write_buffer[2] = {
            0,
            static_cast<uint8_t>(command)
        };

        VERBOSE(get_log_key(), std::format("Start write values 0x{:02X}, 0x{:02X}", write_buffer[0], write_buffer[1]));

        const auto write_result = i2c_master_transmit(
            handle,
            write_buffer, sizeof(write_buffer),
            timeout_ms
        );

        VERBOSE(get_log_key(), std::format("Write result 0x{:02X}", write_result));

        if (write_result != ESP_OK)
        {
            loge(get_log_key(), std::format("Failed to write values 0x{:02X}, 0x{:02X}. [0x{:04X}] {}",
                write_buffer[0], write_buffer[1],
                write_result,
                esp_err_to_name(write_result)));
        }

        return write_result == ESP_OK;
    }
    catch (std::exception& e)
    {
        loge(get_log_key(), std::format("An exception occurred while writing general call 0x{:02X}: {}",
            static_cast<uint8_t>(command),
            e.what()));

        return false;
    }
}
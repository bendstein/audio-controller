//
// Created by bendstein on 12/28/2025.
//
#include "app_common.h"
#include "i2c/i2c.h"
#include "i2c/mcp4725.h"

mcp4725::dac::dac(i2c_master_dev_handle_t device_handle, const uint8_t address) : handle(device_handle), address(address)
{
    log_key = make_log_key();
}

mcp4725::dac::dac(i2c_master_dev_handle_t device_handle, const uint8_t address, const int32_t timeout) : dac(device_handle, address)
{
    timeout_ms = timeout;
    log_key = make_log_key();
}

mcp4725::dac::~dac()
{
    //Remove device from bus on destroy
    if (handle != nullptr)
    {
        try
        {
            FLOGI("{} Removing device 0x{:08X} from bus.",
                 log_key,
                 reinterpret_cast<uintptr_t>(handle));

            if (const auto rm_device_result = i2c_master_bus_rm_device(handle);
                rm_device_result != ESP_OK)
            {
                FLOGE("{} Failed to remove device 0x{:08X} from bus. [0x{:04X}] {}",
                    log_key,
                    reinterpret_cast<uintptr_t>(handle),
                    rm_device_result,
                    esp_err_to_name(rm_device_result));
            }
        }
        catch (std::exception& e)
        {
            FLOGE("{} An exception occurred while removing device 0x{:08X} from bus: {}",
                log_key, reinterpret_cast<uintptr_t>(handle), e.what());
        }
    }
}

std::string mcp4725::dac::make_log_key() const
{
    return std::format("[DAC 0x{:02X}; 0x{:08X}]",
        address,
        reinterpret_cast<uintptr_t>(handle));
}

bool mcp4725::dac::ping() const
{
    FVERBOSE("[{}] Ping", get_log_key());

    return try_general_call(general_call::wake_up);
}

bool mcp4725::dac::try_write_value(const power_down_mode power_mode, const uint16_t value) const
{
    try
    {
        assert(handle != nullptr);
        FVERBOSE("[{}] Write value 0x{:04X}, power mode 0x{:02X}.", get_log_key(), value, static_cast<uint8_t>(power_mode));

        const uint16_t write_buffer_value = (value & 0x0FFF)
            | (static_cast<uint8_t>(power_mode) << 12)
            | (static_cast<uint8_t>(command_type::fast_mode) << 14);

        const uint8_t write_buffer[2] = {
            static_cast<uint8_t>((write_buffer_value & 0xFF00) >> 8),
            static_cast<uint8_t>(write_buffer_value & 0xFF)
        };

        FVERBOSE("[{}] Start write values 0x{:02X}, 0x{:02X}.", get_log_key(), write_buffer[0], write_buffer[1]);

        const auto write_result = i2c_master_transmit(
            handle,
            write_buffer, sizeof(write_buffer),
            timeout_ms
        );

        FVERBOSE("[{}] Write result: 0x{:02X}.", get_log_key(), write_result);

        if (write_result != ESP_OK)
        {
            FLOGE("[{}] Failed to write values 0x{:02X}, 0x{:02X}. [0x{:04X}] {}",
                get_log_key(),
                write_buffer[0], write_buffer[1],
                write_result,
                esp_err_to_name(write_result));
        }

        return write_result == ESP_OK;
    }
    catch (std::exception& e)
    {
        FLOGE("[{}] An exception occurred while writing value 0x{:04X}, power mode 0x{:02X}: {}",
                get_log_key(),
            value, static_cast<uint8_t>(power_mode),
            e.what());

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
        FVERBOSE("[{}] General Call 0x{:02X}.", get_log_key(), static_cast<uint8_t>(command));

        const uint8_t write_buffer[2] = {
            0,
            static_cast<uint8_t>(command)
        };

        FVERBOSE("[{}] Start write values 0x{:02X}, 0x{:02X}.", get_log_key(), write_buffer[0], write_buffer[1]);

        const auto write_result = i2c_master_transmit(
            handle,
            write_buffer, sizeof(write_buffer),
            timeout_ms
        );

        FVERBOSE("[{}] General Call write result: 0x{:02X}.", get_log_key(), write_result);

        if (write_result != ESP_OK)
        {
            FLOGE("[{}] Failed to write values 0x{:02X}, 0x{:02X}. [0x{:04X}] {}",
                get_log_key(),
                write_buffer[0], write_buffer[1],
                write_result,
                esp_err_to_name(write_result));
        }

        return write_result == ESP_OK;
    }
    catch (std::exception& e)
    {
        FLOGE("[{}] An exception occurred while writing general call 0x{:02X}: {}",
            get_log_key(),
            static_cast<uint8_t>(command),
            e.what());

        return false;
    }
}

std::unique_ptr<mcp4725::dac> mcp4725::dac::try_create_on_bus(i2c_master_bus_handle_t bus, const uint8_t addr, const int32_t timeout_ms)
{
    FLOGI("[DAC 0x{:02X}] Creating device on bus.", addr);

    const i2c_device_config_t device_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FAST_HZ,
        .scl_wait_us = I2C_SCL_WAIT_US_DFT,
        .flags = {
            .disable_ack_check = false
        }
    };

    i2c_master_dev_handle_t handle;

    if (const auto add_to_bus_result = i2c_master_bus_add_device(bus, &device_cfg, &handle); add_to_bus_result != ESP_OK)
    {
        FLOGE("[DAC 0x{:02X}] Failed to create device. [0x{:04X}] {}",
            addr, add_to_bus_result, esp_err_to_name(add_to_bus_result));

        return nullptr;
    }

    return std::make_unique<dac>(handle, addr, timeout_ms);
}
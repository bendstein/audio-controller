//
// Created by bendstein on 12/28/2025.
//

#ifndef AUDIO_CONTROLLER_MCP4725_H
#define AUDIO_CONTROLLER_MCP4725_H

#include <format>
#include <driver/i2c_master.h>
#include <driver/i2c_types.h>

#include "i2c/i2c.h"
#include "../audio/notes.h"

namespace mcp4725
{
    enum struct command_type : uint8_t
    {
        fast_mode = 0,
        unused = 1,
        write_dac_register = 2,
        write_dac_register_and_eeprom = 3,
        reserved_0 = 4,
        reserved_1 = 5,
        reserved_2 = 6,
        reserved_3 = 7
    };

    enum struct power_down_mode : uint8_t
    {
        normal = 0,
        pull_down_1k = 1,
        pull_down_100k = 2,
        pull_down_500k = 3,
    };

    enum struct general_call : uint8_t
    {
        reset = 0x06,
        wake_up = 0x09,
    };

    class dac
    {
    public:
        static constexpr uint8_t I2C_ADDR_DFT = 0x62;
        static constexpr int32_t TIMEOUT_MS_DFT = 5000;
    private:
        i2c_master_dev_handle_t handle;
        int32_t timeout_ms = -1;
        uint8_t address;
        std::string log_key;

        [[nodiscard]] std::string make_log_key() const
        {
            return std::format("[DAC 0x{:02X}; 0x{:08X}]",
                address,
                reinterpret_cast<uintptr_t>(handle));
        }
    public:
        dac(i2c_master_dev_handle_t device_handle, const uint8_t address)
            : handle(device_handle), address(address)
        {
            assert(device_handle != nullptr);
            log_key = make_log_key();
        }

        dac(i2c_master_dev_handle_t device_handle, const uint8_t address, const int32_t timeout)
            : dac(device_handle, address)
        {
            timeout_ms = timeout;
            log_key = make_log_key();
        }

        [[nodiscard]] int32_t get_timeout_ms() const { return timeout_ms; }
        void set_timeout(const int32_t timeout) { timeout_ms = timeout; }

        [[nodiscard]] i2c_master_dev_handle_t get_handle() const { return handle; }

        [[nodiscard]] const std::string& get_log_key() const { return log_key; }

        /**
         * @return Whether the dac is alive
         */
        [[nodiscard]] bool ping() const;

        [[nodiscard]] bool try_write_value(power_down_mode power_mode, uint16_t value) const;

        [[nodiscard]] bool try_write_value(uint16_t value) const;

        [[nodiscard]] bool try_general_call(general_call command) const;

        ~dac()
        {
            //Make sure to remove the dac from
            //the i2c bus when it is destroyed.
            try
            {
                if (const auto rm_device_result = i2c_master_bus_rm_device(handle);
                    rm_device_result != ESP_OK)
                {
                    loge(NAMEOF(~dac), std::format("{} Failed to destroy. [0x{:04X}] {}",
                        log_key,
                        rm_device_result,
                        esp_err_to_name(rm_device_result)));
                }
            }
            catch (std::exception& e)
            {
                loge(NAMEOF(~dac), std::format("{} An exception occurred while destroying: {}",
                log_key, e.what()));
            }
        }

        [[nodiscard]] static std::optional<dac> try_create_on_bus(i2c_master_bus_handle_t bus, const uint8_t addr, const int32_t timeout_ms)
        {
            logi(NAMEOF(distance_sensor), std::format("[DAC 0x{:02X}] Creating device on bus.", addr));

            const i2c_device_config_t device_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                // .device_address = static_cast<uint8_t>(addr >> 1),
                .scl_speed_hz = I2C_FAST_HZ,
                .scl_wait_us = I2C_DEV_SCL_WAIT_US,
                .flags = {
                    .disable_ack_check = false
                }
            };

            i2c_master_dev_handle_t handle;

            if (const auto add_to_bus_result = i2c_master_bus_add_device(bus, &device_cfg, &handle); add_to_bus_result != ESP_OK)
            {
                loge(NAMEOF(distance_sensor), std::format("[DAC 0x{:02X}] Failed to create device. [0x{:04X}] {}",
                    addr, add_to_bus_result, esp_err_to_name(add_to_bus_result)));

                return std::nullopt;
            }

            return dac(handle, addr, timeout_ms);
        }
    };

}

#endif //AUDIO_CONTROLLER_MCP4725_H
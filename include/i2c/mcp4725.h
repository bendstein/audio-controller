//
// Created by bendstein on 12/28/2025.
//

#ifndef AUDIO_CONTROLLER_MCP4725_H
#define AUDIO_CONTROLLER_MCP4725_H
#include <memory>

#include <driver/i2c_master.h>

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
        static constexpr uint32_t I2C_SCL_WAIT_US_DFT = 0;
        static constexpr int32_t TIMEOUT_MS_DFT = 5000;
    private:
        i2c_master_dev_handle_t handle;
        int32_t timeout_ms = -1;
        uint8_t address;
        std::string log_key;

        [[nodiscard]] std::string make_log_key() const;
    public:
        dac(i2c_master_dev_handle_t device_handle, uint8_t address);
        dac(i2c_master_dev_handle_t device_handle, uint8_t address, int32_t timeout);

        dac() = delete;
        dac(const dac& other) = delete;
        dac(dac&& other) = delete;
        dac& operator=(const dac& other) = delete;
        dac& operator=(dac&& other) = delete;

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

        ~dac();

        [[nodiscard]] static std::unique_ptr<dac> try_create_on_bus(i2c_master_bus_handle_t bus, uint8_t addr, int32_t timeout_ms);
    };

}

#endif //AUDIO_CONTROLLER_MCP4725_H
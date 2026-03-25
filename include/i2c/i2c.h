//
// Created by bendstein on 12/3/2025.
//

#ifndef AUDIO_CONTROLLER_I2C_H
#define AUDIO_CONTROLLER_I2C_H

#include <soc/gpio_num.h>
#include "app_common.h"

constexpr auto I2C_STANDARD_HZ = 100000;
constexpr auto I2C_FAST_HZ = 400000;
constexpr auto I2C_VERY_FAST_HZ = 1000000; //Non-standard
constexpr auto I2C_HIGH_SPEED_HZ = 34000000;

struct i2c_master_create_cfg
{
    uint8_t port;
    uint8_t scl;
    uint8_t sda;
};

class i2c_master
{
public:
    static constexpr auto GLITCH_CT_DFT = 7;
    static constexpr auto INTERRUPT_PRIORITY_DFT = 0;
    static constexpr auto TRANS_QUEUE_DEPTH_DFT = 0;
    static constexpr auto INTERNAL_PULLUP_DFT = 0;
    static constexpr auto ALLOW_SLEEP_DFT = 0;
private:
    const i2c_master_create_cfg options;
    i2c_master_bus_handle_t handle;
    std::string log_key;

    [[nodiscard]] std::string make_log_key() const
    {
        return std::format("[Bus {} (scl: {}, sda: {}); 0x{:08X}]",
            options.port, options.scl, options.sda,
            reinterpret_cast<uintptr_t>(handle));
    }
public:
    i2c_master(const i2c_master_create_cfg options, i2c_master_bus_handle_t handle)
        : options(options), handle(handle)
    {
        log_key = make_log_key();
    }

    i2c_master() = delete;
    i2c_master(const i2c_master& other) = delete;
    i2c_master(i2c_master&& other) = delete;
    i2c_master& operator=(const i2c_master& other) = delete;
    i2c_master& operator=(i2c_master&& other) = delete;

    ~i2c_master()
    {
        //Delete bus on destroy
        if (handle != nullptr)
        {
            try
            {
                FLOGI("{} Deleting bus.", log_key);

                if (const auto rm_bus_result = i2c_del_master_bus(handle);
                    rm_bus_result != ESP_OK)
                {
                    FLOGE("{} Failed to delete bus. [0x{:04X}] {}",
                        log_key,
                        rm_bus_result,
                        esp_err_to_name(rm_bus_result));
                }
            }
            catch (std::exception& e)
            {
                FLOGE("{} An exception occurred while deleting bus: {}",
                    log_key, e.what());
            }
        }
    }

    [[nodiscard]] i2c_master_bus_handle_t get_handle() const { return handle; }
    [[nodiscard]] i2c_port_t get_port() const { return static_cast<i2c_port_t>(options.port); }
    [[nodiscard]] gpio_num_t get_sda() const { return static_cast<gpio_num_t>(options.sda); }
    [[nodiscard]] gpio_num_t get_scl() const { return static_cast<gpio_num_t>(options.scl); }
    [[nodiscard]] const std::string& get_log_key() const { return log_key; }

    [[nodiscard]]
    static std::unique_ptr<i2c_master> try_create(const i2c_master_create_cfg create_options)
    {
        FLOGI("[Bus {} (scl: {}, sda: {})] Creating bus.",
            create_options.port, create_options.scl, create_options.sda);

        const i2c_master_bus_config_t bus_cfg = {
            .i2c_port = static_cast<i2c_port_t>(create_options.port),
            .sda_io_num = static_cast<gpio_num_t>(create_options.sda),
            .scl_io_num = static_cast<gpio_num_t>(create_options.scl),
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = GLITCH_CT_DFT,
            .intr_priority = INTERRUPT_PRIORITY_DFT,
            .trans_queue_depth = TRANS_QUEUE_DEPTH_DFT,
            .flags = {
                .enable_internal_pullup = INTERNAL_PULLUP_DFT,
                .allow_pd = ALLOW_SLEEP_DFT,
            }
        };

        i2c_master_bus_handle_t handle;
        if (const auto create_result = i2c_new_master_bus(&bus_cfg, &handle);
            create_result != ESP_OK)
        {
            FLOGE("[Bus {} (scl: {}, sda: {})] Failed to create bus. [0x{:04X}] {}",
                create_options.port, create_options.scl, create_options.sda,
                create_result, esp_err_to_name(create_result));

            return nullptr;
        }

        return std::make_unique<i2c_master>(create_options, handle);
    }
};

#endif //AUDIO_CONTROLLER_I2C_H
//
// Created by bendstein on 12/3/2025.
//

#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "i2c.h"

#include <esp_log.h>
#include <format>

#include "app_common.h"

[[nodiscard]]
i2c_master_bus_handle_t i2c_init_bus()
{
    LOGI("i2c", "Init bus");

    constexpr i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_BUS_PORT_0,
        .sda_io_num = I2C_PIN_SDA_0,
        .scl_io_num = I2C_PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = I2C_BUS_GLITCH_CT,
        .intr_priority = I2C_BUS_INTERRUPT_PRIORITY,
        .trans_queue_depth = I2C_BUS_TRANS_QUEUE_DEPTH,
        .flags = {
            .enable_internal_pullup = I2C_BUS_INTERNAL_PULLUP,
            .allow_pd = I2C_BUS_ALLOW_SLEEP
        }
    };

    i2c_master_bus_handle_t handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &handle));

    LOGI("i2c", std::format("Finished initializing I2C bus. Handle: 0x{:08X}",
        reinterpret_cast<uintptr_t>(handle)));

    return handle;
}

[[nodiscard]]
i2c_master_dev_handle_t i2c_init_device(const i2c_master_bus_handle_t bus, uint8_t addr)
{
    LOGI("i2c", std::format("Init I2C device {:04X}.", addr));

    const i2c_device_config_t device_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_DEVICE_SCL_SPEED_HZ,
        .scl_wait_us = I2C_DEVICE_SCL_WAIT_US,
        .flags = {
            .disable_ack_check = false
        }
    };

    i2c_master_dev_handle_t handle;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &device_cfg, &handle));

    LOGI("i2c", std::format("Finished initializing I2C device {:04X}. Handle: 0x{:08X}",
        addr,
        reinterpret_cast<uintptr_t>(handle)));

    return handle;
}
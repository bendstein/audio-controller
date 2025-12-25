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
    logi("i2c", "Init bus");

    gpio_reset_pin(I2C_PIN_SDA_0);
    gpio_reset_pin(I2C_PIN_SCL);
    gpio_set_direction(I2C_PIN_SDA_0, GPIO_MODE_OUTPUT);
    gpio_set_direction(I2C_PIN_SCL, GPIO_MODE_OUTPUT);
    // gpio_set_pull_mode(I2C_PIN_SDA_0, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode(I2C_PIN_SCL, GPIO_PULLUP_ONLY);

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

    logi("i2c", std::format("Finished initializing I2C bus. Handle: 0x{:08X}",
        reinterpret_cast<uintptr_t>(handle)));

    return handle;
}
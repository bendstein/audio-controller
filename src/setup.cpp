//
// Created by bendstein on 12/27/2025.
//
#include "setup.h"

#include "tasks.h"
#include "i2c/i2c.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

i2c_master_bus_handle_t init_i2c_bus(const i2c_port_num_t port, const gpio_num_t sda, const gpio_num_t scl)
{
    logi("setup", "Init bus ");

    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = I2C_BUS_GLITCH_CT,
        .intr_priority = I2C_BUS_INTERRUPT_PRIORITY,
        .trans_queue_depth = I2C_BUS_TRANS_QUEUE_DEPTH,
        .flags = {
            .enable_internal_pullup = I2C_BUS_INTERNAL_PULLUP,
            .allow_pd = I2C_BUS_ALLOW_SLEEP
        }
    };

    gpio_reset_pin(bus_cfg.sda_io_num);
    gpio_reset_pin(bus_cfg.scl_io_num);
    gpio_set_direction(bus_cfg.sda_io_num, GPIO_MODE_OUTPUT);
    gpio_set_direction(bus_cfg.scl_io_num, GPIO_MODE_OUTPUT);

    i2c_master_bus_handle_t handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &handle));

    logi("setup", std::format("Finished initializing I2C bus. Handle: 0x{:08X}",
        reinterpret_cast<uintptr_t>(handle)));

    return handle;
}

void init_distance_sensors(i2c_master_bus_handle_t bus,
    const uint8_t sensor_addresses[], const size_t sensor_count,
    std::optional<std::pair<gp2y0e02b::distance_sensor*, TaskHandle_t>> sensor_tasks[]
)
{
    for (auto i = 0; i < sensor_count; i++)
    {
        sensor_tasks[i] = std::nullopt;

        //Try to create each sensor, apply configuration, and start respective task
        if (const auto maybe_sensor = gp2y0e02b::distance_sensor::try_create_on_bus(
            bus,
            sensor_addresses[i],
            gp2y0e02b::distance_sensor::TIMEOUT_MS_DFT
        ); maybe_sensor.has_value())
        {
            logi("setup", std::format("{} Created handle for distance sensor #{}.", (*maybe_sensor)->get_log_key(), i));

            if (try_configure_gp2y0e02b(*maybe_sensor))
            {
                logi("setup", std::format("{} Successfully configured distance sensor #{}.", (*maybe_sensor)->get_log_key(), i));

                BaseType_t create_task_result;
                TaskHandle_t task_handle;

                if (try_create_distance_sensor_task(
                    std::format("gp2y-{}-{:02X}", i, sensor_addresses[i]),
                    *maybe_sensor,
                    &create_task_result,
                    &task_handle
                ))
                {
                    logi("setup", std::format("{} Started task 0x{:08X} for distance sensor #{}.", (*maybe_sensor)->get_log_key(), reinterpret_cast<uintptr_t>(task_handle), i));
                    sensor_tasks[i] = std::pair(*maybe_sensor, task_handle);
                }
                else
                {
                    loge("setup", std::format("{} Failed to start task for distance sensor #{}. ({})", (*maybe_sensor)->get_log_key(), i, create_task_result));
                }
            }
            else
            {
                loge("setup", std::format("{} Failed to configure distance sensor #{}.", (*maybe_sensor)->get_log_key(), i));
            }
        }
        else
        {
            loge("setup", std::format("[distance sensor 0x{:02X}] Failed to create distance sensor #{}.", sensor_addresses[i], i));
        }
    }
}

void init_dac(i2c_master_bus_handle_t bus, const uint8_t dac_address, std::optional<mcp4725::dac*>* dac, std::optional<TaskHandle_t>* task)
{
    *dac = std::nullopt;
    *task = std::nullopt;

    if (const auto maybe_dac = mcp4725::dac::try_create_on_bus(
        bus,
        dac_address,
        mcp4725::dac::TIMEOUT_MS_DFT); maybe_dac.has_value())
    {
        logi("setup", std::format("{} Created handle for dac.", (*maybe_dac)->get_log_key()));

        BaseType_t create_task_result;
        TaskHandle_t task_handle;

        if (try_create_dac_task(
            std::format("mcp4-{:02X}", dac_address),
            *maybe_dac,
            &create_task_result,
            &task_handle
        ))
        {
            logi("setup", std::format("{} Started task 0x{:08X} for dac.", (*maybe_dac)->get_log_key(), reinterpret_cast<uintptr_t>(task_handle)));
            *dac = maybe_dac;
            *task = task_handle;
        }
        else
        {
            loge("setup", std::format("{} Failed to start task for distance sensor. ({})", (*maybe_dac)->get_log_key(), create_task_result));
        }
    }
    else
    {
        loge("setup", std::format("[DAC 0x{:02X}] Failed to create DAC.", dac_address));
    }
}

bool try_configure_gp2y0e02b(gp2y0e02b::distance_sensor* sensor)
{
    bool success = true;

    logi("setup", std::format("{} Configuring sensor.", sensor->get_log_key()));

    //Perform soft reset of sensor to make sure
    //all settings are at initial default
    if (!sensor->try_soft_reset())
    {
        loge("setup", std::format("{} Failed to perform software reset.", sensor->get_log_key()));
        success = false;
    }

    if (!sensor->try_apply_distance_shift(gp2y0e02b::shift_bit::cm_128))
    {
        loge("setup", std::format("{} Failed to apply distance shift.", sensor->get_log_key()));
        success = false;
    }

    //Return whether all operations succeeded
    return success;
}
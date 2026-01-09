//
// Created by bendstein on 12/27/2025.
//
#include "setup.h"

#include "tasks.h"
#include "i2c/i2c.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

app_state do_setup()
{
    VERBOSE_LOG_STACK_SIZE();

    tone::dft(); //Init static default value for tone at start of program

    auto setup = app_state {
        .i2c_bus_0 = init_i2c_bus(
             I2C_BUS_PORT_0,
             I2C_PIN_SDA_0,
             I2C_PIN_SCL_0
         ),
        .i2c_bus_1 = init_i2c_bus(
            I2C_BUS_PORT_1,
            I2C_PIN_SDA_1,
            I2C_PIN_SCL_1
        ),
        .distance_sensors = {},
        .dac = std::nullopt,
        .wave = new sin_wave_provider(),
        .sensor_tasks = {},
        .dac_task = std::nullopt,
        .piecewise_frequencies = {
            {
                piecewise_frequency_range_breakpoint(),
                piecewise_frequency_range_breakpoint()
            }
        }
    };

    VERBOSE_LOG_STACK_SIZE();

    init_distance_sensors(&setup);
    // init_dac(&setup);

    return setup;
}

i2c_master_bus_handle_t init_i2c_bus(const i2c_port_num_t port, const gpio_num_t sda, const gpio_num_t scl)
{
    logi("setup", std::format("Init bus: port {}, sda {}, scl {}",
        static_cast<int>(port), static_cast<int>(sda), static_cast<int>(scl)));

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

    VERBOSE_LOG_STACK_SIZE();

    gpio_reset_pin(bus_cfg.sda_io_num);
    gpio_reset_pin(bus_cfg.scl_io_num);
    gpio_set_direction(bus_cfg.sda_io_num, GPIO_MODE_OUTPUT);
    gpio_set_direction(bus_cfg.scl_io_num, GPIO_MODE_OUTPUT);

    i2c_master_bus_handle_t handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &handle));

    logi("setup", std::format("Finished initializing I2C bus. Handle: 0x{:08X}",
        reinterpret_cast<uintptr_t>(handle)));

    VERBOSE_LOG_STACK_SIZE();

    return handle;
}

void init_distance_sensors(app_state* setup)
{
    for (auto i = 0; i < SENSORS_COUNT; i++)
    {
        VERBOSE_LOG_STACK_SIZE();

        bool setup_successful = true;

        setup->distance_sensors[i] = std::nullopt;
        setup->sensor_tasks[i] = std::nullopt;

        try
        {
            //Try to create each sensor, apply configuration, and start respective task
            if (auto maybe_sensor = gp2y0e02b::distance_sensor::try_create_on_bus(
                setup->i2c_bus_0,
                SENSOR_ADDRESSES[i],
                gp2y0e02b::distance_sensor::TIMEOUT_MS_DFT
            ); maybe_sensor.has_value())
            {
                auto& sensor = *maybe_sensor;

                VERBOSE_LOG_STACK_SIZE();

                logi("setup", std::format("{} Created handle for distance sensor #{}.", sensor.get_log_key(), i));

                //Make sure device is reachable
                logi("setup", std::format("{} Bus 0 probe.", sensor.get_log_key()));

                const auto probe_result = i2c_master_probe(
                    setup->i2c_bus_0,
                    SENSOR_ADDRESSES[i],
                    gp2y0e02b::distance_sensor::TIMEOUT_MS_DFT);

                if (probe_result != ESP_OK)
                {
                    loge("setup", std::format("{} Probe failed. [0x{:04X}] {}.", sensor.get_log_key(), probe_result, esp_err_to_name(probe_result)));
                    continue;
                }

                VERBOSE_LOG_STACK_SIZE();

                logi("setup", std::format("{} Probe success.", sensor.get_log_key()));

                setup->distance_sensors[i] = maybe_sensor;

                //Configure sensor
                logi("setup", std::format("{} Configuring sensor.", sensor.get_log_key()));

                //Perform soft reset of sensor to make sure
                //all settings are at initial default
                if (!sensor.try_soft_reset())
                {
                    VERBOSE_LOG_STACK_SIZE();

                    setup_successful = false;
                    loge("setup", std::format("{} Failed to perform software reset.", sensor.get_log_key()));
                }

                if (!sensor.try_apply_distance_shift(gp2y0e02b::shift_bit::cm_128))
                {
                    VERBOSE_LOG_STACK_SIZE();

                    setup_successful = false;
                    loge("setup", std::format("{} Failed to apply distance shift.", sensor.get_log_key()));
                }

                if (setup_successful)
                {
                    VERBOSE_LOG_STACK_SIZE();
                    logi("setup", std::format("{} Successfully configured distance sensor #{}.", sensor.get_log_key(), i));

                    BaseType_t create_task_result;

                    if (try_create_distance_sensor_task(
                        std::format("gp2y-{}-{:02X}", i, SENSOR_ADDRESSES[i]),
                        &create_task_result,
                        i,
                        setup
                    ))
                    {
                        logi("setup", std::format("{} Started task 0x{:08X} for distance sensor #{}.", sensor.get_log_key(),
                            reinterpret_cast<uintptr_t>(*setup->sensor_tasks[i]), i));
                    }
                    else
                    {
                        setup_successful = false;
                        loge("setup", std::format("{} Failed to start task for distance sensor #{}. ({})", sensor.get_log_key(),
                            i, create_task_result));
                    }
                }
                else
                {
                    setup_successful = false;
                    loge("setup", std::format("{} Failed to configure distance sensor #{}.", sensor.get_log_key(), i));
                }
            }
            else
            {
                setup_successful = false;
                loge("setup", std::format("[distance sensor 0x{:02X}] Failed to create distance sensor #{}.", SENSOR_ADDRESSES[i], i));
            }
        }
        catch (const std::exception& e)
        {
            setup_successful = false;
            loge("setup", std::format("[distance sensor 0x{:02X}] An exception occurred while attempting to create distance sensor #{}: {}", SENSOR_ADDRESSES[i], i, e.what()));
        }

        VERBOSE_LOG_STACK_SIZE();

        if (!setup_successful)
        {
            //Clear sensor/task if unsuccessful
            if (setup->sensor_tasks[i].has_value())
            {
                if (const auto created_task = *setup->sensor_tasks[i]; created_task != nullptr)
                    vTaskDelete(created_task);

                setup->sensor_tasks[i] = std::nullopt;
            }

            setup->distance_sensors[i] = std::nullopt;
        }
    }
}

void init_dac(app_state* setup)
{
    bool setup_successful = true;

    VERBOSE_LOG_STACK_SIZE();

    try
    {
        if (const auto maybe_dac = mcp4725::dac::try_create_on_bus(
            setup->i2c_bus_1,
            DAC_ADDRESS,
            mcp4725::dac::TIMEOUT_MS_DFT); maybe_dac.has_value())
        {
            VERBOSE_LOG_STACK_SIZE();

            setup->dac = maybe_dac;
            const auto& dac = *maybe_dac;

            logi("setup", std::format("{} Created handle for dac.", dac.get_log_key()));

            BaseType_t create_task_result;

            if (try_create_dac_task(
                std::format("mcp4-{:02X}", DAC_ADDRESS),
                &create_task_result,
                setup
            ))
            {
                logi("setup", std::format("{} Started task 0x{:08X} for dac.", dac.get_log_key(), reinterpret_cast<uintptr_t>(*setup->dac_task)));
            }
            else
            {
                setup_successful = false;
                loge("setup", std::format("{} Failed to start task for dac. ({})", dac.get_log_key(), create_task_result));
            }
        }
        else
        {
            setup_successful = false;
            loge("setup", std::format("[DAC 0x{:02X}] Failed to create DAC.", DAC_ADDRESS));
        }
    }
    catch (const std::exception& e)
    {
        setup_successful = false;
        loge("setup", std::format("[DAC 0x{:02X}] An exception occurred while attempting to create DAC: {}", DAC_ADDRESS, e.what()));
    }

    VERBOSE_LOG_STACK_SIZE();

    if (!setup_successful)
    {
        //Clear dac/task if unsuccessful
        if (setup->dac_task.has_value())
        {
            if (const auto created_task = *setup->dac_task; created_task != nullptr)
                vTaskDelete(created_task);

            setup->dac_task = std::nullopt;
        }

        setup->dac_task = std::nullopt;
    }
}

bool try_configure_gp2y0e02b(gp2y0e02b::distance_sensor* sensor)
{
    try
    {
        VERBOSE_LOG_STACK_SIZE();

        bool success = true;

        logi("setup", std::format("{} Configuring sensor.", sensor->get_log_key()));

        //Perform soft reset of sensor to make sure
        //all settings are at initial default
        if (!sensor->try_soft_reset())
        {
            VERBOSE_LOG_STACK_SIZE();

            loge("setup", std::format("{} Failed to perform software reset.", sensor->get_log_key()));
            success = false;
        }

        if (!sensor->try_apply_distance_shift(gp2y0e02b::shift_bit::cm_128))
        {
            VERBOSE_LOG_STACK_SIZE();

            loge("setup", std::format("{} Failed to apply distance shift.", sensor->get_log_key()));
            success = false;
        }

        VERBOSE_LOG_STACK_SIZE();

        //Return whether all operations succeeded
        return success;
    }
    catch (std::exception& e)
    {
        loge("setup", std::format("{} An exception occurred while configuring sensor: {}", sensor->get_log_key(), e.what()));
        return false;
    }
}

bool try_create_distance_sensor_task(const std::string& task_name, BaseType_t* result_code, const size_t sensor_ndx, app_state* setup)
{
    try
    {
        VERBOSE_LOG_STACK_SIZE();

        if (setup->distance_sensors[sensor_ndx].has_value())
        {
            auto task_param = sensor_task_param {
                .state = setup,
                .index = sensor_ndx
            };

            TaskHandle_t task_handle;

            *result_code = xTaskCreate(
                distance_sensor_task,
                task_name.c_str(),
                DISTANCE_SENSOR_TASK_STACK_SIZE,
                &task_param,
                DISTANCE_SENSOR_TASK_PRIORITY,
                &task_handle
            );

            VERBOSE_LOG_STACK_SIZE();

            setup->sensor_tasks[sensor_ndx] = task_handle;

            if (*result_code != pdPASS)
            {
                loge("setup", std::format("{} Failed to create task. Code: 0x{:04X}.",
                    setup->distance_sensors[sensor_ndx]->get_log_key(),
                    static_cast<int>(*result_code)));
            }

            return *result_code == pdPASS;
        }

        VERBOSE_LOG_STACK_SIZE();

        loge("setup", std::format("Cannot configure task for unconfigured distance sensor #{}.", sensor_ndx));
        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument

        return false;
    }
    catch (std::exception& e)
    {
        loge("setup", std::format("{} An exception occurred while configuring task for distance sensor #{}: {}",
            setup->distance_sensors[sensor_ndx].has_value()? setup->distance_sensors[sensor_ndx]->get_log_key() : "[unconfigured sensor]",
            sensor_ndx,
            e.what()));

        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument
        return false;
    }
}

bool try_create_dac_task(const std::string& task_name, BaseType_t* result_code, app_state* setup)
{
    try
    {
        VERBOSE_LOG_STACK_SIZE();

        VERBOSE("setup", std::format("Creating task {}.", task_name));

        if (setup->dac.has_value())
        {
            auto task_param = dac_task_param {
                .state = setup
            };

            TaskHandle_t task_handle;

            VERBOSE("setup", std::format("Calling xTaskCreate for task {}.", task_name));

            *result_code = xTaskCreate(
                dac_task,
                task_name.c_str(),
                DAC_TASK_STACK_SIZE,
                &task_param,
                DAC_TASK_PRIORITY,
                &task_handle
            );

            VERBOSE_LOG_STACK_SIZE();

            setup->dac_task = task_handle;

            VERBOSE("setup", std::format("Result of xTaskCreate for task {} | Code: 0x{:04X}, Handle: 0x{:08X}",
                task_name,
                static_cast<int>(*result_code),
                reinterpret_cast<uintptr_t>(task_handle)));

            if (*result_code != pdPASS)
            {
                loge("setup", std::format("{} Failed to create task. Code: 0x{:04X}.",
                    setup->dac->get_log_key(),
                    static_cast<int>(*result_code)));
            }

            VERBOSE_LOG_STACK_SIZE();

            return *result_code == pdPASS;
        }

        loge("setup", "Cannot configure task for unconfigured DAC.");
        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument

        VERBOSE_LOG_STACK_SIZE();

        return false;
    }
    catch (std::exception& e)
    {
        loge("setup", std::format("{} An exception occurred while configuring task for DAC: {}",
            setup->dac.has_value()? setup->dac->get_log_key() : "[unconfigured DAC]",
            e.what()));

        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument
        return false;
    }
}
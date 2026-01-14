//
// Created by bendstein on 12/27/2025.
//
#include "setup.h"

#include "tasks.h"
#include "i2c/i2c.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

app_state do_setup()
{
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
        // .dac = nullptr,
        .dac_ctrl = nullptr,
        .wave = new sin_wave_provider(),
        .sensor_tasks = {},
        .dac_write_task = nullptr,
        // .dac_task = nullptr,
        .piecewise_frequencies = {
            {
                piecewise_frequency_range_breakpoint(PIECEWISE_FREQUENCY_BREAKPOINTS[0],
                    single_frequency_range(
                        tone(musical_note::A, 4)
                    )),
                piecewise_frequency_range_breakpoint(PIECEWISE_FREQUENCY_BREAKPOINTS[1], single_frequency_range(*tone::dft()))
            }
        }
    };

    init_distance_sensors(&setup);
    // init_dac(&setup);
    init_dac_controller(&setup);

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

void init_distance_sensors(app_state* setup)
{
    for (auto i = 0; i < SENSORS_COUNT; i++)
    {
        bool setup_successful = true;

        setup->distance_sensors[i] = nullptr;
        setup->sensor_tasks[i] = nullptr;

        try
        {
            //Try to create each sensor, apply configuration, and start respective task
            if (auto maybe_sensor = gp2y0e02b::distance_sensor::try_create_on_bus(
                setup->i2c_bus_0,
                SENSOR_ADDRESSES[i],
                gp2y0e02b::distance_sensor::TIMEOUT_MS_DFT
            ); maybe_sensor != nullptr)
            {
                setup->distance_sensors[i].swap(maybe_sensor);

                //Configure sensor
                logi("setup", std::format("{} Configuring sensor.", setup->distance_sensors[i]->get_log_key()));

                //Perform soft reset of sensor to make sure
                //all settings are at initial default
                if (!setup->distance_sensors[i]->try_soft_reset())
                {
                    setup_successful = false;
                    loge("setup", std::format("{} Failed to perform software reset.", setup->distance_sensors[i]->get_log_key()));
                }

                if (!setup->distance_sensors[i]->try_apply_distance_shift(gp2y0e02b::shift_bit::cm_128))
                {
                    setup_successful = false;
                    loge("setup", std::format("{} Failed to apply distance shift.", setup->distance_sensors[i]->get_log_key()));
                }

                if (setup_successful)
                {
                    logi("setup", std::format("{} Successfully configured distance sensor #{}.", setup->distance_sensors[i]->get_log_key(), i));

                    BaseType_t create_task_result;

                    if (try_create_distance_sensor_task(
                        std::format("gp2y-{}-{:02X}", i, SENSOR_ADDRESSES[i]),
                        &create_task_result,
                        i,
                        setup
                    ))
                    {
                        logi("setup", std::format("{} Started task 0x{:08X} for distance sensor #{}.",
                            setup->distance_sensors[i]->get_log_key(),
                            reinterpret_cast<uintptr_t>(*setup->sensor_tasks[i]), i));
                    }
                    else
                    {
                        setup_successful = false;
                        loge("setup", std::format("{} Failed to start task for distance sensor #{}. ({})",
                            setup->distance_sensors[i]->get_log_key(),
                            i, create_task_result));
                    }
                }
                else
                {
                    setup_successful = false;
                    loge("setup", std::format("{} Failed to configure distance sensor #{}.",
                        setup->distance_sensors[i]->get_log_key(), i));
                }
            }
            else
            {
                setup_successful = false;
                loge("setup", std::format("[distance sensor 0x{:02X}] Failed to create distance sensor #{}.",
                    SENSOR_ADDRESSES[i], i));
            }
        }
        catch (const std::exception& e)
        {
            setup_successful = false;
            loge("setup", std::format("[distance sensor 0x{:02X}] An exception occurred while attempting to create distance sensor #{}: {}",
                SENSOR_ADDRESSES[i], i, e.what()));
        }

        if (!setup_successful)
        {
            //Clear sensor/task if unsuccessful
            if (setup->sensor_tasks[i] != nullptr)
            {
                if (const auto created_task = *setup->sensor_tasks[i]; created_task != nullptr)
                    vTaskDelete(created_task);

                setup->sensor_tasks[i] = nullptr;
            }

            setup->distance_sensors[i] = nullptr;
        }
    }
}

void init_dac_controller(app_state* setup)
{
    bool setup_successful = true;

    try
    {
        if (auto maybe_dac_controller = dac_controller::try_create();
            maybe_dac_controller != nullptr)
        {
            setup->dac_ctrl.swap(maybe_dac_controller);

            logi("setup", std::format("{} Created DAC controller.", dac_controller::LOG_KEY));

            BaseType_t create_task_result;

            if (try_create_dac_write_task(
                "DAC-WRITE",
                &create_task_result,
                setup
            ))
            {
                logi("setup", std::format("{} Started task 0x{:08X} for DAC write.",
                    dac_controller::LOG_KEY,
                    reinterpret_cast<uintptr_t>(*setup->dac_write_task)));
            }
            else
            {
                setup_successful = false;
                loge("setup", std::format("{} Failed to start task for DAC write. ({})",
                    dac_controller::LOG_KEY,
                    create_task_result));
            }

            //Start DAC main task
            if (setup_successful)
            {
                if (const auto dac_controller_start_result = setup->dac_ctrl->start();
                    dac_controller_start_result != start_dac_controller_task_result::ERR)
                {
                    logi("setup", std::format("{} Successfully started DAC controller main task.", dac_controller::LOG_KEY));
                }
                else
                {
                    setup_successful = false;
                    loge("setup", std::format("{} Failed to start DAC controller main task.", dac_controller::LOG_KEY));
                }
            }
        }
        else
        {
            setup_successful = false;
            loge("setup", std::format("{} Failed to create DAC controller.", dac_controller::LOG_KEY));
        }
    }
    catch (const std::exception& e)
    {
        setup_successful = false;
        loge("setup", std::format("{} An exception occurred while attempting to create DAC controller: {}", dac_controller::LOG_KEY, e.what()));
    }

    if (!setup_successful)
    {
        //Clear dac/task if unsuccessful
        if (setup->dac_write_task != nullptr)
        {
            if (const auto created_task = *setup->dac_write_task; created_task != nullptr)
                vTaskDelete(created_task);

            setup->dac_write_task = nullptr;
        }

        setup->dac_ctrl = nullptr;
    }
}

bool try_create_distance_sensor_task(const std::string& task_name, BaseType_t* result_code, const size_t sensor_ndx, app_state* setup)
{
    try
    {
        if (setup->distance_sensors[sensor_ndx] != nullptr)
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

            setup->sensor_tasks[sensor_ndx] = std::make_unique<TaskHandle_t>(task_handle);

            if (*result_code != pdPASS)
            {
                loge("setup", std::format("{} Failed to create task. Code: 0x{:04X}.",
                    setup->distance_sensors[sensor_ndx]->get_log_key(),
                    static_cast<int>(*result_code)));
            }

            return *result_code == pdPASS;
        }

        loge("setup", std::format("Cannot configure task for unconfigured distance sensor #{}.", sensor_ndx));
        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument

        return false;
    }
    catch (std::exception& e)
    {
        loge("setup", std::format("{} An exception occurred while configuring task for distance sensor #{}: {}",
            setup->distance_sensors[sensor_ndx] == nullptr? "[unconfigured sensor]" : setup->distance_sensors[sensor_ndx]->get_log_key(),
            sensor_ndx,
            e.what()));

        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument
        return false;
    }
}

bool try_create_dac_write_task(const std::string& task_name, BaseType_t* result_code, app_state* setup)
{
    try
    {
        VERBOSE("setup", std::format("Creating task {}.", task_name));

        if (setup->dac_ctrl != nullptr)
        {
            auto task_param = dac_write_task_param {
                .state = setup
            };

            TaskHandle_t task_handle;

            VERBOSE("setup", std::format("Calling xTaskCreate for task {}.", task_name));

            *result_code = xTaskCreate(
                dac_write_task,
                task_name.c_str(),
                DAC_TASK_STACK_SIZE,
                &task_param,
                DAC_TASK_PRIORITY,
                &task_handle
            );

            setup->dac_write_task = std::make_unique<TaskHandle_t>(task_handle);

            VERBOSE("setup", std::format("Result of xTaskCreate for task {} | Code: 0x{:04X}, Handle: 0x{:08X}",
                task_name,
                static_cast<int>(*result_code),
                reinterpret_cast<uintptr_t>(task_handle)));

            if (*result_code != pdPASS)
            {
                loge("setup", std::format("{} Failed to create task {}. Code: 0x{:04X}.",
                    dac_controller::LOG_KEY,
                    task_name,
                    static_cast<int>(*result_code)));
            }

            return *result_code == pdPASS;
        }

        loge("setup", "Cannot configure write task for unconfigured DAC controller.");
        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument

        return false;
    }
    catch (std::exception& e)
    {
        loge("setup", std::format("{} An exception occurred while configuring write task for DAC: {}",
            dac_controller::LOG_KEY,
            e.what()));

        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument
        return false;
    }
}

/*
void init_dac(app_state* setup)
{
    bool setup_successful = true;

    try
    {
        if (auto maybe_dac = mcp4725::dac::try_create_on_bus(
            setup->i2c_bus_1,
            DAC_ADDRESS,
            mcp4725::dac::TIMEOUT_MS_DFT); maybe_dac != nullptr)
        {
            setup->dac.swap(maybe_dac);

            logi("setup", std::format("{} Created handle for dac.", setup->dac->get_log_key()));

            BaseType_t create_task_result;

            if (try_create_dac_task(
                std::format("mcp4-{:02X}", DAC_ADDRESS),
                &create_task_result,
                setup
            ))
            {
                logi("setup", std::format("{} Started task 0x{:08X} for dac.", setup->dac->get_log_key(), reinterpret_cast<uintptr_t>(*setup->dac_task)));
            }
            else
            {
                setup_successful = false;
                loge("setup", std::format("{} Failed to start task for dac. ({})", setup->dac->get_log_key(), create_task_result));
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

    if (!setup_successful)
    {
        //Clear dac/task if unsuccessful
        if (setup->dac_task != nullptr)
        {
            if (const auto created_task = *setup->dac_task; created_task != nullptr)
                vTaskDelete(created_task);

            setup->dac_task = nullptr;
        }

        setup->dac_task = nullptr;
    }
}

bool try_create_dac_task(const std::string& task_name, BaseType_t* result_code, app_state* setup)
{
    try
    {
        VERBOSE("setup", std::format("Creating task {}.", task_name));

        if (setup->dac != nullptr)
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

            setup->dac_task = std::make_unique<TaskHandle_t>(task_handle);

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

            return *result_code == pdPASS;
        }

        loge("setup", "Cannot configure task for unconfigured DAC.");
        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument

        return false;
    }
    catch (std::exception& e)
    {
        loge("setup", std::format("{} An exception occurred while configuring task for DAC: {}",
            setup->dac == nullptr? "[unconfigured DAC]" : setup->dac->get_log_key(),
            e.what()));

        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument
        return false;
    }
}
*/
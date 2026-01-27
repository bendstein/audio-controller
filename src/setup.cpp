//
// Created by bendstein on 12/27/2025.
//
#include "setup.h"

#include "tasks.h"
#include "i2c/i2c.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

app_state do_setup()
{
    auto setup = app_state {
        .i2c_buses = {},
        .dac_ctrl = nullptr,
        .dac_write_task = nullptr,
        .distance_sensors = {},
        .sensor_tasks = {},
        .current_tones = {}
    };

    for (auto& current_tone : setup.current_tones)
        current_tone = musical_note_tone::create_invalid();

    init_i2c_buses(&setup);
    init_distance_sensors(&setup);
    init_dac_controller(&setup);

    return setup;
}

void init_i2c_buses(app_state* app_state)
{
    for (auto i = 0; i < BUS_COUNT; i++)
    {
        const auto bus_cfg = BUS_OPTIONS[i];

        try
        {
            if (auto maybe_bus = i2c_master::try_create(bus_cfg);
                maybe_bus != nullptr)
            {
                app_state->i2c_buses[i].swap(maybe_bus);
                FLOGI("{} Successfully created bus.", app_state->i2c_buses[i]->get_log_key());
            }
            else
            {
                FLOGE("[Bus {}] Bus was not successfully created.",
                    bus_cfg.port, bus_cfg.scl, bus_cfg.sda);
            }
        }
        catch (std::exception& e)
        {
            FLOGE("[Bus {} (scl: {}, sda: {})] An exception occurred while creating bus: {}",
                bus_cfg.port, bus_cfg.scl, bus_cfg.sda, e.what());
        }
    }
}

void init_distance_sensors(app_state* setup)
{
    //Init each distance sensor
    for (auto i = 0; i < SENSORS_COUNT; i++)
    {
        bool setup_successful = true;
        const auto [ addr, bus_num ] = SENSOR_CFG[i];

        auto& sensors = setup->distance_sensors[bus_num];
        const auto sensor_index = sensors.grow();

        if (bus_num >= BUS_COUNT || setup->i2c_buses[bus_num] == nullptr)
        {
            FLOGE("[distance sensor 0x{:02X}] Cannot create distance sensor #{}, as bus {} doesn't exist.", addr, i, bus_num);
            continue;
        }

        try
        {
            //Try to create each sensor, apply configuration, and start respective task
            if (auto maybe_sensor = gp2y0e02b::distance_sensor::try_create_on_bus(
                setup->i2c_buses[bus_num]->get_handle(),
                addr,
                i,
                gp2y0e02b::distance_sensor::TIMEOUT_MS_DFT
            ); maybe_sensor != nullptr)
            {
                sensors[sensor_index].swap(maybe_sensor);

                //Configure sensor
                FLOGI("{} Configuring sensor.", sensors[sensor_index]->get_log_key());

                //Perform soft reset of sensor to make sure
                //all settings are at initial default
                if (!sensors[sensor_index]->try_soft_reset())
                {
                    setup_successful = false;
                    FLOGE("{} Failed to perform software reset.", sensors[sensor_index]->get_log_key());
                }

                if (!sensors[sensor_index]->try_apply_distance_shift(gp2y0e02b::shift_bit::cm_128))
                {
                    setup_successful = false;
                    FLOGE("{} Failed to apply distance shift.", sensors[sensor_index]->get_log_key());
                }

                if (setup_successful)
                {
                    FLOGI("{} Successfully configured distance sensor #{}.", sensors[sensor_index]->get_log_key(), i);
                }
                else
                {
                    setup_successful = false;
                    FLOGE("{} Failed to configure distance sensor #{}.", sensors[sensor_index]->get_log_key(), i);
                }
            }
            else
            {
                setup_successful = false;
                FLOGE("[distance sensor 0x{:02X}] Failed to create distance sensor #{}.", addr, i);
            }
        }
        catch (const std::exception& e)
        {
            setup_successful = false;
            FLOGE("[distance sensor 0x{:02X}] An exception occurred while attempting to create distance sensor #{}: {}",
                addr, i, e.what());
        }

        //Remove sensor if not successfully set up
        if (!setup_successful)
        {
            sensors.remove_last();
        }
    }

    //Setup tasks for each bus
    for (auto i = 0; i < BUS_COUNT; i++)
    {
        FLOGI("Creating distance sensor task for I2C bus {}.", i);

        if (setup->distance_sensors[i].empty())
        {
            FLOGW("I2C bus {} has no distance sensors.", i);
            continue;
        }

        BaseType_t create_task_result;
        if (const auto init_task_success = try_create_distance_sensor_task(std::format("<i2c-bus-0x{:02X}>", i), &create_task_result, i, setup);
            init_task_success)
        {
            FLOGI("Successfully created distance sensor task 0x{:08X} for I2C bus {}: 0x{:04X}",
                reinterpret_cast<uintptr_t>(*setup->sensor_tasks[i]),
                i,
                create_task_result);
        }
        else
        {
            FLOGE("Failed to create distance sensor task for I2C bus {}: 0x{:04X}", i, create_task_result);
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

            FLOGI("{} Created DAC controller.", dac_controller::LOG_KEY);

            BaseType_t create_task_result;

            if (try_create_dac_write_task(
                "<DAC-WRITE>",
                &create_task_result,
                setup
            ))
            {
                FLOGI("{} Started task 0x{:08X} for DAC write.",
                    dac_controller::LOG_KEY,
                    reinterpret_cast<uintptr_t>(*setup->dac_write_task));
            }
            else
            {
                setup_successful = false;
                FLOGE("{} Failed to start task for DAC write. ({})",
                    dac_controller::LOG_KEY,
                    create_task_result);
            }
        }
        else
        {
            setup_successful = false;
            FLOGE("{} Failed to create DAC controller.", dac_controller::LOG_KEY);
        }
    }
    catch (const std::exception& e)
    {
        setup_successful = false;
        FLOGE("{} An exception occurred while attempting to create DAC controller: {}", dac_controller::LOG_KEY, e.what());
    }

    if (!setup_successful)
    {
        //Clear dac/task if unsuccessful
        // if (setup->dac_write_task != nullptr)
        // {
        //     if (const auto created_task = *setup->dac_write_task; created_task != nullptr)
        //         vTaskDelete(created_task);
        //
        //     setup->dac_write_task = nullptr;
        // }

        setup->dac_ctrl = nullptr;
    }
}

bool try_create_distance_sensor_task(const std::string& task_name, BaseType_t* result_code, const size_t bus_ndx, app_state* setup)
{
    try
    {
        if (!setup->distance_sensors[bus_ndx].empty())
        {
            auto task_param = bus_sensor_task_param {
                .state = setup,
                .index = bus_ndx
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

            setup->sensor_tasks[bus_ndx] = std::make_unique<TaskHandle_t>(task_handle);

            if (*result_code != pdPASS)
            {
                FLOGE("Failed to create distance sensor task for I2C bus {}. Code: 0x{:04X}.",
                    bus_ndx,
                    static_cast<int>(*result_code));
            }

            return *result_code == pdPASS;
        }

        FLOGE("Cannot configure task for I2C bus {}, as it has no sensors.", bus_ndx);
        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument

        return false;
    }
    catch (std::exception& e)
    {
        FLOGE("An exception occurred while configuring distance sensor task for bus {}: {}",
            bus_ndx,
            e.what());

        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument
        return false;
    }
}

bool try_create_dac_write_task(const std::string& task_name, BaseType_t* result_code, app_state* setup)
{
    try
    {
        FVERBOSE("Creating task {}.", task_name);

        if (setup->dac_ctrl != nullptr)
        {
            auto task_param = dac_write_task_param {
                .state = setup
            };

            TaskHandle_t task_handle;

            FVERBOSE("Calling xTaskCreate for task {}.", task_name);

            *result_code = xTaskCreate(
                dac_write_task,
                task_name.c_str(),
                DAC_WRITE_TASK_STACK_SIZE,
                &task_param,
                DAC_WRITE_TASK_PRIORITY,
                &task_handle
            );

            setup->dac_write_task = std::make_unique<TaskHandle_t>(task_handle);

            FVERBOSE("Result of xTaskCreate for task {} | Code: 0x{:04X}, Handle: 0x{:08X}",
                task_name,
                static_cast<int>(*result_code),
                reinterpret_cast<uintptr_t>(task_handle));

            if (*result_code != pdPASS)
            {
                FLOGE("{} Failed to create task {}. Code: 0x{:04X}.",
                    dac_controller::LOG_KEY,
                    task_name,
                    static_cast<int>(*result_code));
            }

            return *result_code == pdPASS;
        }

        LOGE("Cannot configure write task for unconfigured DAC controller.");
        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument

        return false;
    }
    catch (std::exception& e)
    {
        FLOGE("{} An exception occurred while configuring write task for DAC: {}",
            dac_controller::LOG_KEY,
            e.what());

        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument
        return false;
    }
}
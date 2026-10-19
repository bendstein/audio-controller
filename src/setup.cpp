//
// Created by bendstein on 12/27/2025.
//
#include "setup.h"
#include "tasks.h"
#include "i2c/i2c.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

app_state_controller start_controller()
{
    LOGI("Starting (Controller).");
    LOGI("Performing Setup...");

    auto state = app_state_controller {
        .i2c_buses = {},
        .distance_sensors = {},
        .sensor_tasks = {},
        .send_audio_task = nullptr,
        .current_tones = {},
        .current_distances = {}
    };

    for (auto& current_tone : state.current_tones)
        current_tone = musical_note_tone_volume::create_invalid();

    init_i2c_buses(&state);
    init_distance_sensors(&state);

    BaseType_t create_task_result;

    if (try_create_send_audio_task(
        "<SEND-AUDIO>",
        &create_task_result,
        &state
    ))
    {
        FLOGI("{} Started task 0x{:08X} for send audio.",
            signal_generator::LOG_KEY,
            reinterpret_cast<uintptr_t>(*state.send_audio_task));
    }
    else
    {
        FLOGE("{} Failed to start task for send audio. ({})",
            signal_generator::LOG_KEY,
            create_task_result);
    }

    LOGI("Finished Setup.");

    return state;
}

app_state_audio start_audio()
{
    LOGI("Starting (Audio).");
    LOGI("Performing Setup...");

    auto state = app_state_audio {
        .signal_gtor = nullptr,
        .receive_audio_task = nullptr
    };

    init_signal_generator(&state);

    BaseType_t create_task_result;

    if (try_create_receive_audio_task(
        "<RECV-AUDIO>",
        &create_task_result,
        &state
    ))
    {
        FLOGI("{} Started task 0x{:08X} for receive audio.",
            signal_generator::LOG_KEY,
            reinterpret_cast<uintptr_t>(*state.receive_audio_task));
    }
    else
    {
        FLOGE("{} Failed to start task for receive audio. ({})",
            signal_generator::LOG_KEY,
            create_task_result);
    }

    LOGI("Finished Setup.");

    return state;
}

void init_i2c_buses(app_state_controller* state)
{
    for (auto i = 0; i < BUS_COUNT; i++)
    {
        const auto bus_cfg = BUS_OPTIONS[i];

        try
        {
            if (auto maybe_bus = i2c_master::try_create(bus_cfg);
                maybe_bus != nullptr)
            {
                state->i2c_buses[i].swap(maybe_bus);
                FLOGI("{} Successfully created bus.", state->i2c_buses[i]->get_log_key());
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

void init_distance_sensors(app_state_controller* state)
{
    //Init each distance sensor
    for (auto i = 0; i < SENSORS_COUNT; i++)
    {
        bool setup_successful = true;
        const auto [ addr, bus_num ] = SENSOR_CFG[i];

        auto& sensors = state->distance_sensors[bus_num];
        const auto sensor_index = sensors.grow();

        if (bus_num >= BUS_COUNT || state->i2c_buses[bus_num] == nullptr)
        {
            FLOGE("[distance sensor 0x{:02X}] Cannot create distance sensor #{}, as bus {} doesn't exist.", addr, i, bus_num);
            continue;
        }

        try
        {
            //Try to create each sensor, apply configuration, and start respective task
            if (auto maybe_sensor = gp2y0e02b::distance_sensor::try_create_on_bus(
                state->i2c_buses[bus_num]->get_handle(),
                addr,
                i,
                gp2y0e02b::distance_sensor::TIMEOUT_MS_DFT
            ); maybe_sensor != nullptr)
            {
                sensors[sensor_index].swap(maybe_sensor);

                //Configure sensor
                FLOGI("{} Configuring sensor.", sensors[sensor_index]->get_log_key());

                // //Try ping
                // FLOGI("{} Pinging sensor.", sensors[sensor_index]->get_log_key());
                // if (!sensors[sensor_index]->ping())
                // {
                //     setup_successful = false;
                //     FLOGE("{} Failed to ping sensor.", sensors[sensor_index]->get_log_key());
                // }

                //Perform soft reset of sensor to make sure
                //all settings are at initial default
                // if (setup_successful && !sensors[sensor_index]->try_soft_reset())
                if (!sensors[sensor_index]->try_soft_reset())
                {
                    setup_successful = false;
                    FLOGE("{} Failed to perform software reset.", sensors[sensor_index]->get_log_key());
                }

                if (setup_successful && !sensors[sensor_index]->try_apply_distance_shift(gp2y0e02b::shift_bit::cm_128))
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
    auto all_empty = true;

    for (auto i = 0; i < BUS_COUNT; i++)
    {
        FLOGI("Creating distance sensor task for I2C bus {}.", i);

        if (state->distance_sensors[i].empty())
        {
            FLOGW("I2C bus {} has no distance sensors.", i);
            continue;
        }

        all_empty = false;

        BaseType_t create_task_result;
        if (const auto init_task_success = try_create_distance_sensor_task(std::format("<I2C-BUS-0x{:02X}>", i), &create_task_result, i, state);
            init_task_success)
        {
            FLOGI("Successfully created distance sensor task 0x{:08X} for I2C bus {}: 0x{:04X}",
                reinterpret_cast<uintptr_t>(*state->sensor_tasks[i]),
                i,
                create_task_result);
        }
        else
        {
            FLOGE("Failed to create distance sensor task for I2C bus {}: 0x{:04X}", i, create_task_result);
        }
    }

    //If every bus is empty, log error
    if (all_empty)
        FLOGE("Failed to setup distance sensor tasks. All ({}) buses are empty of distance sensors.", BUS_COUNT);
}

void init_signal_generator(app_state_audio* state)
{
    bool setup_successful = true;

    try
    {
        if (auto maybe_signal_generator = signal_generator::try_create();
            maybe_signal_generator != nullptr)
        {
            state->signal_gtor.swap(maybe_signal_generator);

            FLOGI("{} Created signal generator.", signal_generator::LOG_KEY);
        }
        else
        {
            setup_successful = false;
            FLOGE("{} Failed to create signal generator.", signal_generator::LOG_KEY);
        }
    }
    catch (const std::exception& e)
    {
        setup_successful = false;
        FLOGE("{} An exception occurred while attempting to create signal generator: {}", signal_generator::LOG_KEY, e.what());
    }

    if (!setup_successful)
    {
        state->signal_gtor = nullptr;
    }
}

bool try_create_distance_sensor_task(const std::string& task_name, BaseType_t* result_code, const size_t bus_ndx, app_state_controller* state)
{
    try
    {
        FLOGI("Creating task {}.", task_name);

        if (!state->distance_sensors[bus_ndx].empty())
        {
            auto task_param = bus_sensor_task_param {
                .state = state,
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

            state->sensor_tasks[bus_ndx] = std::make_unique<TaskHandle_t>(task_handle);

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

bool try_create_send_audio_task(const std::string& task_name, BaseType_t* result_code, app_state_controller* state)
{
    try
    {
        FLOGI("Creating task {}.", task_name);
        auto task_param = send_audio_task_param {
            .state = state
        };

        TaskHandle_t task_handle;

        FLOGD("Calling xTaskCreate for task {}.", task_name);

        *result_code = xTaskCreate(
            send_audio_task,
            task_name.c_str(),
            SEND_AUDIO_TASK_STACK_SIZE,
            &task_param,
            SEND_AUDIO_TASK_PRIORITY,
            &task_handle
        );

        state->send_audio_task = std::make_unique<TaskHandle_t>(task_handle);

        FLOGD("Result of xTaskCreate for task {} | Code: 0x{:04X}, Handle: 0x{:08X}",
            task_name,
            static_cast<int>(*result_code),
            reinterpret_cast<uintptr_t>(task_handle));

        if (*result_code != pdPASS)
        {
            FLOGE("{} Failed to create task {}. Code: 0x{:04X}.",
                signal_generator::LOG_KEY,
                task_name,
                static_cast<int>(*result_code));
        }

        return *result_code == pdPASS;
    }
    catch (std::exception& e)
    {
        FLOGE("{} An exception occurred while configuring audio send task: {}",
            signal_generator::LOG_KEY,
            e.what());

        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument
        return false;
    }
}


bool try_create_receive_audio_task(const std::string& task_name, BaseType_t* result_code, app_state_audio* state)
{
    try
    {
        FLOGI("Creating task {}.", task_name);

        if (state->signal_gtor != nullptr)
        {
            auto task_param = receive_audio_task_param {
                .state = state
            };

            TaskHandle_t task_handle;

            FLOGD("Calling xTaskCreate for task {}.", task_name);

            *result_code = xTaskCreate(
                receive_audio_task,
                task_name.c_str(),
                RECEIVE_AUDIO_TASK_STACK_SIZE,
                &task_param,
                RECEIVE_AUDIO_TASK_PRIORITY,
                &task_handle
            );

            state->receive_audio_task = std::make_unique<TaskHandle_t>(task_handle);

            FLOGD("Result of xTaskCreate for task {} | Code: 0x{:04X}, Handle: 0x{:08X}",
                task_name,
                static_cast<int>(*result_code),
                reinterpret_cast<uintptr_t>(task_handle));

            if (*result_code != pdPASS)
            {
                FLOGE("{} Failed to create task {}. Code: 0x{:04X}.",
                    signal_generator::LOG_KEY,
                    task_name,
                    static_cast<int>(*result_code));
            }

            return *result_code == pdPASS;
        }

        LOGE("Cannot configure receive audio task for unconfigured application.");
        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument

        return false;
    }
    catch (std::exception& e)
    {
        FLOGE("{} An exception occurred while configuring audio receive task: {}",
            signal_generator::LOG_KEY,
            e.what());

        *result_code = pdFREERTOS_ERRNO_EINVAL; //Invalid argument
        return false;
    }
}
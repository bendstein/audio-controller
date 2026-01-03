//
// Created by bendstein on 12/27/2025.
//
#include "setup.h"

#include "tasks.h"
#include "i2c/i2c.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

setup_data do_setup()
{
    auto setup = setup_data {
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
        .sensors = {},
        .dac = std::nullopt,
        .musical_distance_sensors = {},
        .sensor_tasks = {},
        .dac_task = std::nullopt
    };

    init_distance_sensors(&setup);
    init_musical_distance_sensors(&setup);
    init_dac(&setup);

    return setup;
}


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

void init_distance_sensors(setup_data* setup)
{
    for (auto i = 0; i < SENSORS_COUNT; i++)
    {
        setup->sensors[i] = std::nullopt;
        setup->sensor_tasks[i] = std::nullopt;

        //Try to create each sensor, apply configuration, and start respective task
        if (const auto maybe_sensor = gp2y0e02b::distance_sensor::try_create_on_bus(
            setup->i2c_bus_0,
            SENSOR_ADDRESSES[i],
            gp2y0e02b::distance_sensor::TIMEOUT_MS_DFT
        ); maybe_sensor.has_value())
        {
            logi("setup", std::format("{} Created handle for distance sensor #{}.", (*maybe_sensor)->get_log_key(), i));

            if (try_configure_gp2y0e02b(*maybe_sensor))
            {
                logi("setup", std::format("{} Successfully configured distance sensor #{}.", (*maybe_sensor)->get_log_key(), i));

                BaseType_t create_task_result;

                if (try_create_distance_sensor_task(
                    std::format("gp2y-{}-{:02X}", i, SENSOR_ADDRESSES[i]),
                    &create_task_result,
                    i,
                    setup
                ))
                {
                    logi("setup", std::format("{} Started task 0x{:08X} for distance sensor #{}.", (*maybe_sensor)->get_log_key(), reinterpret_cast<uintptr_t>(*setup->sensor_tasks[i]), i));
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
            loge("setup", std::format("[distance sensor 0x{:02X}] Failed to create distance sensor #{}.", SENSOR_ADDRESSES[i], i));
        }
    }
}

void init_musical_distance_sensors(setup_data* setup)
{
    //Define frequency ranges for each sensor
    const piecewise_frequency_range_breakpoint* breakpoints[SENSORS_COUNT] = {
        new piecewise_frequency_range_breakpoint[PIECEWISE_FREQUENCY_BREAKPOINT_COUNT]
        {
            {
                .breakpoint = PIECEWISE_FREQUENCY_BREAKPOINTS[0],
                .range = new single_frequency_range(new tone(musical_note::C, 4))
            },
            {
                .breakpoint = PIECEWISE_FREQUENCY_BREAKPOINTS[1],
                .range = new single_frequency_range(new tone(musical_note::C_Sharp, 4))
            }
        }
    };

    //Add each sensor with the sound data for each segment of length
    for (auto i = 0; i < SENSORS_COUNT; i++)
    {
        if (const auto maybe_sensor = setup->sensors[i]; maybe_sensor.has_value())
        {
            const auto sensor = *maybe_sensor;
            const auto sound_data = new piecewise_frequency_range(breakpoints[i], PIECEWISE_FREQUENCY_BREAKPOINT_COUNT);
            setup->musical_distance_sensors[i] = new musical_distance_sensor(sensor, sound_data);
        }
    }
}

void init_dac(setup_data* setup)
{
    if (const auto maybe_dac = mcp4725::dac::try_create_on_bus(
        setup->i2c_bus_1,
        DAC_ADDRESS,
        mcp4725::dac::TIMEOUT_MS_DFT); maybe_dac.has_value())
    {
        logi("setup", std::format("{} Created handle for dac.", (*maybe_dac)->get_log_key()));

        BaseType_t create_task_result;

        if (try_create_dac_task(
            std::format("mcp4-{:02X}", DAC_ADDRESS),
            &create_task_result,
            setup
        ))
        {
            logi("setup", std::format("{} Started task 0x{:08X} for dac.", (*maybe_dac)->get_log_key(), reinterpret_cast<uintptr_t>(*setup->dac_task)));
        }
        else
        {
            loge("setup", std::format("{} Failed to start task for distance sensor. ({})", (*maybe_dac)->get_log_key(), create_task_result));
        }
    }
    else
    {
        loge("setup", std::format("[DAC 0x{:02X}] Failed to create DAC.", DAC_ADDRESS));
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

bool try_create_distance_sensor_task(const std::string& task_name, BaseType_t* result_code, const size_t sensor_ndx, setup_data* setup)
{
    if (const auto maybe_sensor = setup->sensors[sensor_ndx];
        maybe_sensor.has_value())
    {
        const auto sensor = *maybe_sensor;

        TaskHandle_t task_handle;

        *result_code = xTaskCreate(
            distance_sensor_task,
            task_name.c_str(),
            DISTANCE_SENSOR_TASK_STACK_SIZE,
            sensor,
            DISTANCE_SENSOR_TASK_PRIORITY,
            &task_handle
        );

        setup->sensor_tasks[sensor_ndx] = task_handle;
        return *result_code == pdPASS;
    }

    return false;
}

bool try_create_dac_task(const std::string& task_name, BaseType_t* result_code, setup_data* setup)
{
    if (setup->dac.has_value())
    {
        const auto dac = *setup->dac;

        auto task_param = dac_task_param {

            .dac = dac,
            .wave = new sin_wave_provider(),
            .musical_distance_sensors = {}
        };

        for (auto i = 0; i < SENSORS_COUNT; i++)
            task_param.musical_distance_sensors[i] = setup->musical_distance_sensors[i];

        TaskHandle_t task_handle;

        *result_code = xTaskCreate(
            dac_task,
            task_name.c_str(),
            DAC_TASK_STACK_SIZE,
            &task_param,
            DAC_TASK_PRIORITY,
            &task_handle
        );

        setup->dac_task = task_handle;

        return *result_code == pdPASS;
    }

    return false;
}
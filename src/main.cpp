#include "main.h"
#include "i2c/gp2y0e02b/distance_sensor.h"
#include "setup.h"
#include "driver/gpio.h"

[[noreturn]]
void app_main()
{
    //Set default log level for all
    esp_log_level_set("*", ESP_LOG_INFO);

    //If VERBOSE is set, enable verbose logging
    if (FLAG_VERBOSE)
    {
        esp_log_level_set(LOG_VERBOSE_TAG, ESP_LOG_VERBOSE);
    }

#ifdef CFG_GP2Y0E02B_I2C_ADDR
    //configure_gp2y0e02b never returns, so rest of program is never executed when configuring a sensor
    configure_gp2y0e02b();
#endif

    try
    {
        // const auto setup_data = do_setup();

        TaskHandle_t log_tasks_task = nullptr;

        if (FLAG_VERBOSE)
        {
            VERBOSE(NAMEOF(app_main), "Creating log_tasks_task.");

            if (const auto create_log_tasks_task_result = xTaskCreate(log_system_state_task, "log_sys_state",
                4096,  nullptr, 1, &log_tasks_task); create_log_tasks_task_result != pdPASS)
            {
                loge(NAMEOF(app_main), std::format("Failed to create log_tasks_task. (0x{:04X})", create_log_tasks_task_result));
            }

            VERBOSE(NAMEOF(app_main), "Successfully created log_tasks_task.");
        }

        const auto state = do_setup();

        // const auto bus = init_i2c_bus(
        //     I2C_BUS_PORT_0,
        //     I2C_PIN_SDA_0,
        //     I2C_PIN_SCL_0
        // );
        //
        // if (const auto maybe_sensor = gp2y0e02b::distance_sensor::try_create_on_bus(bus, 0x80, 60000);
        //     maybe_sensor == nullptr)
        // {
        //     loge(NAMEOF(app_main), "Failed to create sensor on bus.");
        // }
        // else
        // {
        //     logi(NAMEOF(app_main), maybe_sensor->get_log_key());
        // }

        for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
        {
            vTaskDelay(250 / portTICK_PERIOD_MS);
        }
    }
    catch (const std::exception& e)
    {
        loge(NAMEOF(app_main), std::format("An exception occurred: {}", e.what()));
    }

    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    } //Make sure to never return
}

#ifdef CFG_GP2Y0E02B_I2C_ADDR
[[noreturn]]
void configure_gp2y0e02b()
{
    try
    {
        const auto bus = i2c_init_bus();
        gpio_reset_pin(gp2y0e02b::distance_sensor::PIN_VPP_ENABLE);
        gpio_set_direction(gp2y0e02b::distance_sensor::PIN_VPP_ENABLE, GPIO_MODE_OUTPUT);

        gp2y0e02b::distance_sensor::permanently_apply_new_i2c_address(bus, CFG_GP2Y0E02B_I2C_ADDR);
    }
    catch (const std::exception& e)
    {
        loge(NAMEOF(configure_gp2y0e02b), std::format("An exception occurred while configuring sensor address: {}",
            e.what()));
    }

    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    } //Make sure to never return
}
#endif

#if configCHECK_FOR_STACK_OVERFLOW != 0

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    loge("SO-HOOK", std::format("A stack overflow occurred in task {} (0x:{:08X})",
        pcTaskName,
        reinterpret_cast<uintptr_t>(xTask)));
}

#endif
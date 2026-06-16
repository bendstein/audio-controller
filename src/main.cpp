#include "main.h"
#include "i2c/gp2y0e02b/distance_sensor.h"
#include "setup.h"
#include "driver/gpio.h"
#include "i2c_drivers/i2c.h"

[[noreturn]]
void app_main()
{
    //Set default log level for all
    esp_log_level_set("*", ESP_LOG_DEBUG);

    if constexpr (FLAG_VERBOSE) //Enable verbose logging for specific tags
    {
        esp_log_level_set("dac_controller.h", ESP_LOG_VERBOSE);
        // esp_log_level_set("tasks.cpp", ESP_LOG_VERBOSE);
    }

    LOGV("Log level V enabled.");
    LOGD("Log level D enabled.");
    LOGI("Log level I enabled.");
    LOGW("Log level W enabled.");
    LOGE("Log level E enabled.");

    //Setup background logging task if configured
    if constexpr (FLAG_VERBOSE)
    {
        try
        {
            TaskHandle_t log_tasks_task = nullptr;

            VERBOSE("Creating log_tasks_task.");

            if (const auto create_log_tasks_task_result = xTaskCreate(log_system_state_task, "<LOG_SYS_STATE>",
                4096,  nullptr, 1, &log_tasks_task); create_log_tasks_task_result != pdPASS)
            {
                FLOGE("Failed to create log_tasks_task. (0x{:04X})", create_log_tasks_task_result);
            }

            VERBOSE("Successfully created log_tasks_task.");
        }
        catch (const std::exception& e)
        {
            LOGEX(e);
        }
    }

    FLOGI("Main method type: {}", MAIN_METHOD_TYPE_NAME);

    //Choose which main method to execute based on flags
    if constexpr (MAIN_METHOD_TYPE == main_method_type::configure_gp2y0e02b)
        main_configure_gp2y0e02b();
    else if constexpr (MAIN_METHOD_TYPE == main_method_type::gp2y0e02b_interface)
        main_gp2y0e02b_interface();
    else
        main_standard();
}

void main_standard()
{
    try
    {
        const auto state = do_setup();

        for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
        {
            vTaskDelay(250 / portTICK_PERIOD_MS);
        }
    }
    catch (const std::exception& e)
    {
        LOGEX(e);
    }

    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    } //Make sure to never return
}

void main_configure_gp2y0e02b()
{
    try
    {
        LOGI("Start configuring gp2y0e02b distance sensors.");

        //TODO
        // const auto bus = i2c_init_bus();
        // gpio_reset_pin(gp2y0e02b::distance_sensor::PIN_VPP_ENABLE);
        // gpio_set_direction(gp2y0e02b::distance_sensor::PIN_VPP_ENABLE, GPIO_MODE_OUTPUT);
        //
        // gp2y0e02b::distance_sensor::permanently_apply_new_i2c_address(bus, CFG_GP2Y0E02B_I2C_ADDR);
    }
    catch (const std::exception& e)
    {
        LOGEX(e);
    }

    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    } //Make sure to never return
}

void main_gp2y0e02b_interface()
{
    try
    {
        LOGI("Start parallel interface for gp2y0e02b distance sensors.");

        const i2c::device devs_0[] = {
            i2c::device({
                .address = 0x80
            })
        };

        const i2c::bus buses[] = {
            i2c::bus({
                .sda = GPIO_NUM_12,
                .scl = GPIO_NUM_13
            }, devs_0, std::size(devs_0))
        };

        const auto ctrl = i2c::controller(buses, std::size(buses));

        std::string error_message = "";

        while (true)
        {
            if (const auto start_result = ctrl.get_bus(GPIO_NUM_12)->send_start_condition(); start_result.is_err(error_message))
                FLOGE("Failed to send start condition. {}", error_message);

            vTaskDelay(500 / portTICK_PERIOD_MS);

            if (const auto stop_result = ctrl.get_bus(GPIO_NUM_12)->send_stop_condition(); stop_result.is_err(error_message))
                FLOGE("Failed to send stop condition. {}", error_message);

            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
    catch (const std::exception& e)
    {
        LOGEX(e);
    }

    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    } //Make sure to never return
}

#if configCHECK_FOR_STACK_OVERFLOW != 0

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    loge("SO-HOOK", std::format("A stack overflow occurred in task {} (0x:{:08X})",
        pcTaskName,
        reinterpret_cast<uintptr_t>(xTask)));
}

#endif
#include "driver/gpio.h"

#include "main.h"
#include "setup.h"
#include "audio/signal_generator.h"

[[noreturn]]
void app_main()
{
    //Set default log level for all
    esp_log_level_set("*", ESP_LOG_DEBUG);

    if constexpr (FLAG_VERBOSE) //Enable verbose logging for specific tags
    {
        // esp_log_level_set("dac_controller.h", ESP_LOG_VERBOSE);
        // esp_log_level_set("tasks.cpp", ESP_LOG_VERBOSE);
    }

    LOGV("Log level V enabled.");
    LOGD("Log level D enabled.");
    LOGI("Log level I enabled.");
    LOGW("Log level W enabled.");
    LOGE("Log level E enabled.");

    //Call correct main method
    if constexpr (MAIN_METHOD_TYPE == main_method_type::configure_gp2y0e02b)
    {
        main_configure_gp2y0e02b();
    }
    else if constexpr (MAIN_METHOD_TYPE == main_method_type::signal_generator)
    {
        main_signal_generator();
    }
    else
    {
        main_standard();
    }
}

void main_standard()
{
    try
    {
        const auto state = start_standard();

        while (true)
        {
            vTaskDelay(portMAX_DELAY);
        } //Make sure to never return
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
        //TODO

        // const auto bus = i2c_init_bus();
        // gpio_reset_pin(gp2y0e02b::distance_sensor::PIN_VPP_ENABLE);
        // gpio_set_direction(gp2y0e02b::distance_sensor::PIN_VPP_ENABLE, GPIO_MODE_OUTPUT);
        //
        // gp2y0e02b::distance_sensor::permanently_apply_new_i2c_address(bus, CFG_GP2Y0E02B_I2C_ADDR);

        while (true)
        {
            vTaskDelay(portMAX_DELAY);
        } //Make sure to never return
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

void main_signal_generator()
{
    try
    {
        const auto state = start_signal_generator();

        while (true)
        {
            vTaskDelay(portMAX_DELAY);
        } //Make sure to never return
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
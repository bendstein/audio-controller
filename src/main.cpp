#include "app_common.h"
#include "i2c/gp2y0e02b/distance_sensor.h"
#include "setup.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #define CFG_GP2Y0E02B_I2C_ADDR (0x10)

#ifdef CFG_GP2Y0E02B_I2C_ADDR
[[noreturn]]
void configure_gp2y0e02b();
#endif

extern "C" {
    [[noreturn]]
    void app_main(void);
}

[[noreturn]]
void app_main()
{
    //Set default log level for all
    esp_log_level_set("*", ESP_LOG_INFO);

    //If VERBOSE is set, enable verbose logging for appropriate tags
#ifdef LOG_VERBOSE
    esp_log_level_set(LOG_VERBOSE_TAG, ESP_LOG_VERBOSE);
    esp_log_level_set(LOG_VERBOSE_STACK_SIZE_TAG, ESP_LOG_VERBOSE);
#endif

#ifdef CFG_GP2Y0E02B_I2C_ADDR
    //configure_gp2y0e02b never returns, so rest of program is never executed when configuring a sensor
    configure_gp2y0e02b();
#endif

    try
    {
        // gpio_reset_pin(PIN_LED_BUILTIN);
        // gpio_set_direction(PIN_LED_BUILTIN, GPIO_MODE_OUTPUT);
        // gpio_set_level(PIN_LED_BUILTIN, HIGH);

        const auto setup_data = do_setup();

        for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
        {
            vTaskDelay(100 / portTICK_PERIOD_US);

            if (i % 100 == 0)
            {
                logd("main_task", std::format("Stack watermark: {}",
                    uxTaskGetStackHighWaterMark2(nullptr)));
            }
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
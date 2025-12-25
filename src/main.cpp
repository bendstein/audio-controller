#include "app_common.h"
#include "i2c.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "gp2y0e02b.h"
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
#ifdef CFG_GP2Y0E02B_I2C_ADDR
    //configure_gp2y0e02b never returns, so rest of program is never executed when configuring a sensor
    configure_gp2y0e02b();
#endif

    try
    {
        const auto i2c_bus = i2c_init_bus();

        const auto maybe_sensor_0 = gp2y0e02b::distance_sensor::try_create_on_bus(
            i2c_bus,
            gp2y0e02b::distance_sensor::I2C_ADDR_DFT,
            5000
        );

        if (!maybe_sensor_0.has_value())
            throw std::runtime_error("Failed to create sensor");

        const auto sensor_0 = *maybe_sensor_0;

        //Get initial distance shift
        while (!sensor_0->try_update_distance_shift())
        {
            loge(NAMEOF(app_main), "Failed to read distance shift.");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        logi(NAMEOF(app_main), std::format("Distance shift: {}",
            static_cast<uint8_t>(sensor_0->get_distance_shift())));
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        //Read distance in a loop
        while (true)
        {
            uint8_t distance = 0;
            if (sensor_0->try_update_distance(&distance))
            {
                logi(NAMEOF(app_main), std::format("Current distance: {}", distance));
            }
            else
            {
                loge(NAMEOF(app_main), "Failed to read sensor distance.");
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
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
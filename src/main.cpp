#include "main.h"
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
void configure_gp2y0e02b(i2c_master_bus_handle_t bus);
#endif

extern "C" {
    [[noreturn]]
    void app_main(void);
}

/*
 * TODO:
 *  - Finish and test I2C address config for distance sensors
 *  - Literally everything else
 */
[[noreturn]]
void app_main()
{
    try
    {
        const auto i2c_bus = i2c_init_bus();

#ifdef CFG_GP2Y0E02B_I2C_ADDR
    //configure_gp2y0e02b never returns, so rest of program is never executed
    //when configuring a sensor
    configure_gp2y0e02b(i2c_bus);
#endif

        const auto sensor_0 = i2c_init_device(
            i2c_bus,
            0x80,
            i2c_device_type::GP2Y0E02B);

        while (true)
        {
            const auto maybe_value = gp2y0e02b::read_from_register(
                &sensor_0,
                gp2y0e02b::register_map_tag::DISTANCE_3_THRU_0,
                5000
            ).and_then([](const gp2y0e02b::register_map_entry entry)
            {
                return std::optional(entry.data.distance_3_thru_0);
            });

            if (maybe_value.has_value())
            {
                logi("app_main", std::format("Active/Standby: {}",
                    static_cast<uint8_t>(maybe_value->distance_part)));
            }
            else
            {
                logw("app_main", "Active/Standby register query returned no result.");
            }

            // auto success = i2c_ping_device(&sensor_0, 5000);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    catch (const std::exception& e)
    {
        loge("app_main", std::format("An exception occurred: {}", e.what()));
    }

    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    } //Make sure to never return
}

#ifdef CFG_GP2Y0E02B_I2C_ADDR
#include "gp2y0e02b.h"

[[noreturn]]
void configure_gp2y0e02b(const i2c_master_bus_handle_t bus)
{
    try
    {
        gpio_reset_pin(GP2Y0E02B_PIN_ENABLE_VPP);
        gpio_set_direction(GP2Y0E02B_PIN_ENABLE_VPP, GPIO_MODE_OUTPUT);

        gp2y0e02b::set_i2c_addr(bus, CFG_GP2Y0E02B_I2C_ADDR);
    }
    catch (const std::exception& e)
    {
        logE("app_main", std::format("An exception occurred while configuring sensor address: {}",
            e.what()));
    }

    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    } //Make sure to never return
}
#endif
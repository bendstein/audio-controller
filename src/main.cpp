#include "main.h"
#include "app_common.h"
#include "i2c.h"

#include "driver/gpio.h"
#include "esp_log.h"
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

        auto sensor_0 = i2c_init_device(
            i2c_bus,
            0x80,
            GP2Y0E02B);

        while (true)
        {
            auto success = i2c_ping_device(&sensor_0, 5000);
            vTaskDelay(portMAX_DELAY);
        }
    }
    catch (const std::exception& e)
    {
        LOGE("app_main", std::format("An exception occurred: {}", e.what()));
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
        LOGE("app_main", std::format("An exception occurred while configuring sensor address: {}",
            e.what()));
    }

    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    } //Make sure to never return
}
#endif
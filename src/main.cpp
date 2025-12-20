#include "main.h"
#include "app_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "i2c.h"

// #define CFG_GP2Y0E02B_I2C_ADDR (0x10)

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

        // const auto sensor_0 = i2c_init_device(i2c_bus, 0x80);

        while (true)
        {
            LOGI("main", std::format("i2c bus: {:#010X}",
                reinterpret_cast<uintptr_t>(i2c_bus)
            ));
        }
    }
    catch (const std::exception& e)
    {
        LOGE("app_main", std::format("An exception occurred: {}", e.what()));
    }

    while (true) {} //Make sure to never return
}

#ifdef CFG_GP2Y0E02B_I2C_ADDR
#include "sensor_cfg.h"

[[noreturn]]
void configure_gp2y0e02b(const i2c_master_bus_handle_t bus)
{
    try
    {
        gpio_reset_pin(PIN_ENABLE_VPP);
        gpio_set_direction(PIN_ENABLE_VPP, GPIO_MODE_OUTPUT);

        set_gp2y0e02b_i2c_addr(bus, CFG_GP2Y0E02B_I2C_ADDR);
    }
    catch (const std::exception& e)
    {
        LOGE("app_main", std::format("An exception occurred while configuring sensor address: {}",
            e.what()));
    }

    while (true) {}
}
#endif
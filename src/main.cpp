#include "main.h"
#include "app_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "i2c.h"

#define CFG_GP2Y0E02B_I2C_ADDR (1)

extern "C" {
    [[noreturn]]
    void app_main(void);
}

[[noreturn]]
void app_main()
{
    const auto i2c_bus = i2c_init_bus();

#ifdef CFG_GP2Y0E02B_I2C_ADDR
    configure_gp2y0e02b(i2c_bus);
#endif

    while (true)
    {
        ESP_LOGI("main", "a");
    }
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
    catch (...)
    {
        ESP_LOGE("main", "Failed to program sensor address.");
    }

    while (true) {}
}
#endif
#include "app_common.h"
#include "i2c/gp2y0e02b/distance_sensor.h"
#include "setup.h"
#include "tasks.h"

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
#ifdef CFG_GP2Y0E02B_I2C_ADDR
    //configure_gp2y0e02b never returns, so rest of program is never executed when configuring a sensor
    configure_gp2y0e02b();
#endif

    try
    {
        gpio_reset_pin(PIN_LED_BUILTIN);
        gpio_set_direction(PIN_LED_BUILTIN, GPIO_MODE_OUTPUT);
        gpio_set_level(PIN_LED_BUILTIN, HIGH);

        constexpr size_t SENSORS_COUNT = 1;
        constexpr uint8_t sensor_addresses[SENSORS_COUNT] = {
            gp2y0e02b::distance_sensor::I2C_ADDR_DFT
        };

        const auto i2c_bus_0 = init_i2c_bus(
            I2C_BUS_PORT_0,
            I2C_PIN_SDA_0,
            I2C_PIN_SCL_0
        );

        const auto i2c_bus_1 = init_i2c_bus(
            I2C_BUS_PORT_1,
            I2C_PIN_SDA_1,
            I2C_PIN_SCL_1
        );

        std::optional<std::pair<gp2y0e02b::distance_sensor*, TaskHandle_t>> sensors_tasks[SENSORS_COUNT] = {};
        std::optional<mcp4725::dac*> dac = std::nullopt;
        std::optional<TaskHandle_t> dac_task = std::nullopt;

        init_distance_sensors(i2c_bus_0, sensor_addresses, SENSORS_COUNT, sensors_tasks);
        init_dac(i2c_bus_1, mcp4725::dac::I2C_ADDR_DFT, &dac, &dac_task);

        while (true)
        {
            for (auto i = 0; i < SENSORS_COUNT; i++)
            {
                if (const auto maybe_sensor = sensors_tasks[i];
                    maybe_sensor.has_value())
                {
                    const auto sensor = maybe_sensor->first;
                    const auto sensor_task = maybe_sensor->second;

                    if (i > 0 && (i % 100) == 0)
                    {
                        logi(NAMEOF(app_main),
                            std::format("uxTaskGetStackHighWaterMark {}: {}",
                                i,
                                uxTaskGetStackHighWaterMark(sensor_task)));
                    }

                    uint8_t distance = sensor->get_distance();
                    gpio_set_level(PIN_LED_BUILTIN, DIGITAL(distance < 64));
                    logi(NAMEOF(app_main), std::format("Current distance: {}", distance));
                }
                else
                {
                    logw(NAMEOF(app_main),
                        std::format("No sensor {}", i));
                }
            }

            vTaskDelay(100 / portTICK_PERIOD_US);
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
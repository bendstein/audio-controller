#include "app_common.h"
#include "i2c.h"
#include "setup.h"
#include "tasks.h"

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

        gpio_reset_pin(PIN_LED_BUILTIN);
        gpio_set_direction(PIN_LED_BUILTIN, GPIO_MODE_OUTPUT);
        gpio_set_level(PIN_LED_BUILTIN, HIGH);

        constexpr size_t SENSORS_COUNT = 1;
        constexpr uint8_t sensor_addresses[SENSORS_COUNT] = {
            gp2y0e02b::distance_sensor::I2C_ADDR_DFT
        };
        std::optional<gp2y0e02b::distance_sensor*> sensors[SENSORS_COUNT] = {};
        std::optional<TaskHandle_t> sensor_tasks[SENSORS_COUNT] = {};

        for (auto i = 0; i < SENSORS_COUNT; i++)
        {
            const auto maybe_sensor = gp2y0e02b::distance_sensor::try_create_on_bus(
                i2c_bus,
                sensor_addresses[i],
                5000
            );
            sensors[i] = maybe_sensor;

            if (maybe_sensor.has_value())
            {
                if (const auto sensor = *maybe_sensor; try_configure_gp2y0e02b(sensor))
                {
                    logi(NAMEOF(app_main),
                        std::format("Successfully configured sensor {}", i));

                    BaseType_t create_task_result;
                    TaskHandle_t task_handle;

                    if (try_create_distance_sensor_task(
                        std::format("gp2y-{:02X}", i),
                        sensor,
                        &create_task_result,
                        &task_handle
                    ))
                    {
                        logi(NAMEOF(app_main),
                            std::format("Successfully created task for sensor {}", i));
                        sensor_tasks[i] = task_handle;
                    }
                    else
                    {
                        loge(NAMEOF(app_main),
                            std::format("Failed to create task for sensor {}", i));
                        sensor_tasks[i] = std::nullopt;
                    }
                }
                else
                {
                    loge(NAMEOF(app_main),
                        std::format("Failed to configure sensor {}", i));
                    sensor_tasks[i] = std::nullopt;
                }
            }
            else
            {
                loge(NAMEOF(app_main),
                    std::format("Failed to init sensor {}", i));
                sensor_tasks[i] = std::nullopt;
            }
        }

        while (true)
        {
            for (auto i = 0; i < SENSORS_COUNT; i++)
            {
                if (const auto maybe_sensor = sensors[i];
                    maybe_sensor.has_value())
                {
                    if (i > 0 && (i % 100) == 0)
                    {
                        const auto sensor_task = *(sensor_tasks[i]);
                        logi(NAMEOF(app_main),
                            std::format("uxTaskGetStackHighWaterMark {}: {}",
                                i,
                                uxTaskGetStackHighWaterMark(sensor_task)));
                    }

                    const auto sensor = *maybe_sensor;
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
//
// Created by bendstein on 1/10/2026.
//

#ifndef AUDIO_CONTROLLER_MAIN_H
#define AUDIO_CONTROLLER_MAIN_H
#include <format>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_common.h"

// #define CFG_GP2Y0E02B_I2C_ADDR (0x10)

[[noreturn]]
void main_configure_gp2y0e02b();

[[noreturn]]
void main_gp2y0e02b_interface();

[[noreturn]]
void main_standard();

extern "C" {
    [[noreturn]]
    void app_main(void);
}

[[noreturn]]
inline void log_system_state_task(void*)
{
    if constexpr (FLAG_VERBOSE)
    {
        static char list_tasks_buffer[1024] {};

        do
        {
            try
            {
                vTaskDelay(5000 / portTICK_PERIOD_MS);

                vTaskList(list_tasks_buffer);
                VERBOSE_LB(list_tasks_buffer);
            }
            catch (std::exception &e)
            {
                LOGEX(e);
            }
        }
        while (true);
    }
    else
    {
        do { vTaskDelay(portMAX_DELAY); } while (true);
    }
}

#endif //AUDIO_CONTROLLER_MAIN_H
#include <string>
#include <iomanip>

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
        esp_log_level_set("main.cpp", ESP_LOG_VERBOSE);
        // esp_log_level_set("uart_receive_queue.cpp", ESP_LOG_VERBOSE);
        esp_log_level_set("tasks.cpp", ESP_LOG_VERBOSE);
        // esp_log_level_set("signal_generator.cpp", ESP_LOG_VERBOSE);
    }

    LOGV("Log level V enabled.");
    LOGD("Log level D enabled.");
    LOGI("Log level I enabled.");
    LOGW("Log level W enabled.");
    LOGE("Log level E enabled.");

    //Setup background logging task if in verbose mode
    TaskHandle_t log_sys_state_task = nullptr;
    if constexpr (FLAG_VERBOSE)
    {
        VERBOSE("Creating task LOG-SYS-STATE.");

        if (const auto create_log_sys_state_task_result = xTaskCreate(log_system_state_task, "<LOG-SYS-STATE>",
            4096, nullptr, 1, &log_sys_state_task); create_log_sys_state_task_result != pdPASS)
        {
            FLOGE("Failed to create task LOG-SYS-STATE. (0x{:04X})", create_log_sys_state_task_result);
        }
        else
        {
            VERBOSE("Successfully created task LOG-SYS-STATE.");
        }
    }

    //Call correct main method
    if constexpr (MAIN_METHOD_TYPE == main_method_type::configure_gp2y0e02b)
    {
        main_configure_gp2y0e02b();
    }
    else if constexpr(MAIN_METHOD_TYPE == main_method_type::audio)
    {
        main_audio();
    }
    else
    {
        main_controller();
    }
}

void main_controller()
{
    try
    {
        const auto state = start_controller();

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

void main_audio()
{
    try
    {
        const auto state = start_audio();

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

void log_system_state_task(void*)
{
    if constexpr (FLAG_VERBOSE)
    {
        constexpr size_t MAX_TASK_STATUSES = 16;
        static TaskStatus_t task_statuses[MAX_TASK_STATUSES] {};

        do
        {
            try
            {
                vTaskDelay(2500 / portTICK_PERIOD_MS);

                uint32_t total_runtime = 0;
                const auto tasks_count = uxTaskGetSystemState(task_statuses, MAX_TASK_STATUSES, &total_runtime);

                if (tasks_count > MAX_TASK_STATUSES)
                    FVERBOSE("Number of tasks ({}) exceeds the expected max of {}.", tasks_count, MAX_TASK_STATUSES);

                const auto task_statuses_len = std::min(MAX_TASK_STATUSES, tasks_count);

                if (task_statuses_len == 0)
                {
                    VERBOSE("No tasks are running. (?)");
                    continue;
                }

                const auto total_runtime_percent = total_runtime / 100;

                //Build and output task data table
                constexpr uint8_t COL_SIZE_NAME = configMAX_TASK_NAME_LEN;
                constexpr uint8_t COL_SIZE_N = 2;
                constexpr uint8_t COL_SIZE_STATE = 9;
                constexpr uint8_t COL_SIZE_PRIORITY = 8;
                constexpr uint8_t COL_SIZE_STACK_SIZE = 15;
                constexpr uint8_t COL_SIZE_RUNTIME = 10;
                constexpr uint8_t COL_SIZE_HANDLE = 10;

                std::stringstream ss {};

                ss << std::setw(COL_SIZE_NAME) << std::left << "Name" << std::setw(0) << " | ";
                ss << std::setw(COL_SIZE_N) << std::left << "N" << std::setw(0) << " | ";
                ss << std::setw(COL_SIZE_STATE) << std::left << "State" << std::setw(0) << " | ";
                ss << std::setw(COL_SIZE_PRIORITY) << std::left << "Priority" << std::setw(0) << " | ";
                ss << std::setw(COL_SIZE_STACK_SIZE) << std::left << "Remaining Stack" << std::setw(0) << " | ";
                ss << std::setw(COL_SIZE_RUNTIME) << std::left << "% Run Time" << std::setw(0) << " | ";
                ss << std::setw(COL_SIZE_HANDLE) << std::left << "Handle" << std::setw(0);
                ss << std::endl;

                //Separator between table header and body
                ss.fill('-');

                ss << std::setw(COL_SIZE_NAME) << "" << std::setw(0) << " | ";
                ss << std::setw(COL_SIZE_N) << "" << std::setw(0) << " | ";
                ss << std::setw(COL_SIZE_STATE) << "" << std::setw(0) << " | ";
                ss << std::setw(COL_SIZE_PRIORITY) << "" << std::setw(0) << " | ";
                ss << std::setw(COL_SIZE_STACK_SIZE) << "" << std::setw(0) << " | ";
                ss << std::setw(COL_SIZE_RUNTIME) << "" << std::setw(0) << " | ";
                ss << std::setw(COL_SIZE_HANDLE) << "" << std::setw(0);
                ss << std::endl;

                ss.fill(' ');

                for (size_t i = 0; i < task_statuses_len; i++)
                {
                    const auto task_status = task_statuses[i];

                    ss << std::setw(COL_SIZE_NAME) << std::left << task_status.pcTaskName << std::setw(0) << " | ";
                    ss << std::setw(COL_SIZE_N) << std::right << task_status.xTaskNumber << std::setw(0) << " | ";

                    ss << std::setw(COL_SIZE_STATE) << std::left;

                    switch (task_status.eCurrentState)
                    {
                        case eRunning:
                            ss << "Running";
                            break;
                        case eReady:
                            ss << "Ready";
                            break;
                        case eBlocked:
                            ss << "Blocked";
                            break;
                        case eSuspended:
                            ss << "Suspended";
                            break;
                        case eDeleted:
                            ss << "Deleted";
                            break;
                        case eInvalid:
                        default:
                            ss << "Unknown";
                            break;
                    }

                    ss << std::setw(0) << " | ";

                    ss << std::setw(COL_SIZE_PRIORITY) << std::right << task_status.uxCurrentPriority << std::setw(0) << " | ";
                    ss << std::setw(COL_SIZE_STACK_SIZE) << std::right << task_status.usStackHighWaterMark << std::setw(0) << " | ";

                    if (total_runtime_percent > 0)
                        ss << std::setw(COL_SIZE_RUNTIME) << std::right << std::format("{}%", task_status.ulRunTimeCounter / total_runtime_percent) << std::setw(0) << " | ";
                    else
                        ss << std::setw(COL_SIZE_RUNTIME) << std::left << "N/A" << std::setw(0) << " | ";

                    ss << std::setw(COL_SIZE_HANDLE) << std::left << std::format("0x{:08X}", reinterpret_cast<uintptr_t>(task_status.xHandle)) << std::setw(0);

                    ss << std::endl;
                }

                VERBOSE_LB(ss.str());
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

#if configCHECK_FOR_STACK_OVERFLOW != 0

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    loge("SO-HOOK", std::format("A stack overflow occurred in task {} (0x:{:08X})",
        pcTaskName,
        reinterpret_cast<uintptr_t>(xTask)));
}

#endif
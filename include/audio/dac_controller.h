//
// Created by bendstein on 1/13/2026.
//

#ifndef AUDIO_CONTROLLER_DAC_CONTROLLER_H
#define AUDIO_CONTROLLER_DAC_CONTROLLER_H

#include <atomic>
#include <format>
#include <memory>
#include <string>
#include <utility>
#include <driver/dac_continuous.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_common.h"
#include "i2c/mcp4725.h"

enum struct start_dac_controller_task_result
{
    OK = 0,
    ERR = 1,
    ALREADY_STARTED = 2
};

class dac_controller
{
public:
    static constexpr std::string LOG_KEY = "[DAC Ctrl]";
private:
    static constexpr std::string TASK_NAME = "DAC-DMA";
    static constexpr int TASK_STACK_SIZE = 4096;
    static constexpr int TASK_PRIORITY = 1;

    static constexpr int DAC_CFG_DESC_NUM = 6;
    static constexpr size_t DAC_CFG_BUFFER_SIZE = 2048;
    static constexpr int DAC_CFG_SAMPLE_RATE = 1000;
    static constexpr int DAC_CFG_OFFSET = 0;

    static constexpr size_t DAC_DOUBLE_BUFFER_SIZE = 512;

    dac_continuous_handle_t handle;
    TaskHandle_t task_handle;

    std::mutex active_double_buffer_mutex {};
    uint8_t* active_buffer;
    uint8_t* working_buffer;
    size_t active_buffer_length = 0;
    size_t working_buffer_length = 0;

    static bool IRAM_ATTR dac_on_convert_done_cb(dac_continuous_handle_t handle, const dac_event_data_t* event, void* user_data)
    {
        //TODO
    }

    [[noreturn]]
    static void task(void* task_params)
    {
        assert(task_params != nullptr);
        const auto self = static_cast<dac_controller*>(task_params);

        do
        {
            //TODO
            vTaskDelay(10 / portTICK_PERIOD_MS);
        } while (true);
    }

    void flush_working_buffer()
    {
        //Swap buffers so that the current working buffer becomes the active buffer
        std::lock_guard lock {active_double_buffer_mutex};
        std::swap(working_buffer, active_buffer);
        active_buffer_length = working_buffer_length;
        working_buffer_length = 0;
    }
public:

    explicit dac_controller(dac_continuous_handle_t handle)
        : handle(handle), task_handle(nullptr)
    {
        active_buffer = new uint8_t[DAC_CFG_BUFFER_SIZE];
        working_buffer = new uint8_t[DAC_CFG_BUFFER_SIZE];
    }
    dac_controller(const dac_controller& other) = delete;

    dac_controller(dac_controller&& other) = delete;

    dac_controller& operator=(const dac_controller& other) = delete;

    dac_controller& operator=(dac_controller&& other) = delete;

    start_dac_controller_task_result start()
    {
        if (task_handle == nullptr)
            return start_dac_controller_task_result::ALREADY_STARTED;

        try
        {
            if (const auto result = xTaskCreate(
                task,
                TASK_NAME.c_str(),
                TASK_STACK_SIZE,
                this,
                TASK_PRIORITY,
                &task_handle
            ); result != pdPASS)
            {
                loge(NAMEOF(dac_controller), std::format("{} Failed to create task. Code: 0x{:04X}.",
                     LOG_KEY,
                     static_cast<int>(result)));
                return start_dac_controller_task_result::ERR;
            }

            if (task_handle == nullptr)
            {
                loge(NAMEOF(dac_controller), std::format("{} Failed to create task. No task handle was created.",
                    LOG_KEY));

                return start_dac_controller_task_result::ERR;
            }

            return start_dac_controller_task_result::OK;
        }
        catch (std::exception& e)
        {
            loge(NAMEOF(dac_controller), std::format("{} An exception occurred while creating task: {}",
                LOG_KEY,
                e.what()));

            return start_dac_controller_task_result::ERR;
        }
    }

    void accept_frequencies(float frequencies[], size_t size)
    {
        //TODO
    }

    [[nodiscard]] static std::unique_ptr<dac_controller> try_create(const dac_channel_mask_t channels = DAC_CHANNEL_MASK_ALL,const  dac_continuous_channel_mode_t channel_mode = DAC_CHANNEL_MODE_SIMUL)
    {
        logi(NAMEOF(dac_controller), std::format("{} Creating DAC controller.", LOG_KEY));

        dac_continuous_handle_t handle;

        const dac_continuous_config_t cfg = {
            .chan_mask = channels,
            .desc_num = DAC_CFG_DESC_NUM,
            .buf_size = DAC_CFG_BUFFER_SIZE,
            .freq_hz = DAC_CFG_SAMPLE_RATE,
            .offset = DAC_CFG_OFFSET,
            .clk_src = DAC_DIGI_CLK_SRC_APLL,
            .chan_mode = channel_mode
        };

        if (const auto alloc_result = dac_continuous_new_channels(&cfg, &handle);
            alloc_result != ESP_OK)
        {
            loge(NAMEOF(dac_controller), std::format("{} Failed to allocate DAC channels. [0x{:04X}] {}",
                LOG_KEY, alloc_result, esp_err_to_name(alloc_result)));

            return nullptr;
        }

        return std::make_unique<dac_controller>(handle);
    }

    ~dac_controller()
    {
        delete[] active_buffer;
        delete[] working_buffer;

        //Dealloc DAC channel handle, associated task handle on destroy
        if (handle != nullptr)
        {
            try
            {
                logi(NAMEOF(~dac_controller), std::format("{} Removing DAC channel handle 0x{:08X}.",
                     LOG_KEY,
                     reinterpret_cast<uintptr_t>(handle)));

                if (const auto remove_dac_channel_result = dac_continuous_del_channels(handle);
                    remove_dac_channel_result != ESP_OK)
                {
                    loge(NAMEOF(~dac_controller), std::format("{} Failed to remove DAC channel handle 0x{:08X}. [0x{:04X}] {}",
                        LOG_KEY,
                        reinterpret_cast<uintptr_t>(handle),
                        remove_dac_channel_result,
                        esp_err_to_name(remove_dac_channel_result)));
                }
            }
            catch (std::exception& e)
            {
                loge(NAMEOF(~dac_controller), std::format("{} An exception occurred while removing DAC channel handle 0x{:08X}: {}",
                    LOG_KEY, reinterpret_cast<uintptr_t>(handle), e.what()));
            }
        }
    }
};

#endif //AUDIO_CONTROLLER_DAC_CONTROLLER_H
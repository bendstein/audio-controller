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

class dac_controller
{
public:
    static constexpr std::string LOG_KEY = "[DAC Ctrl]";
private:
    static constexpr int DAC_CFG_DESC_NUM = 8;
    static constexpr size_t DAC_CFG_BUFFER_SIZE = 1 << 11;
    static constexpr int DAC_CFG_SAMPLE_RATE = 1 << 14;
    static constexpr int DAC_CFG_OFFSET = 0;

    static constexpr size_t DAC_DOUBLE_BUFFER_SIZE = 1 << 13;
    static constexpr float DAC_DOUBLE_BUFFER_TIME_SECONDS = static_cast<float>(DAC_DOUBLE_BUFFER_SIZE) / DAC_CFG_SAMPLE_RATE;
    static constexpr size_t MOVING_AVERAGE_LEN_BETWEEN_BUFFERS = 4;
    static constexpr size_t MOVING_AVERAGE_LEN = 4;

    dac_continuous_handle_t handle;
    uint8_t* active_buffer;
    uint8_t* working_buffer;

    void swap_buffers()
    {
        std::swap(active_buffer, working_buffer);
    }
public:

    explicit dac_controller(dac_continuous_handle_t handle)
        : handle(handle)
    {
        active_buffer = new uint8_t[DAC_DOUBLE_BUFFER_SIZE] {};
        working_buffer = new uint8_t[DAC_DOUBLE_BUFFER_SIZE] {};
    }

    dac_controller(const dac_controller& other) = delete;

    dac_controller(dac_controller&& other) = delete;

    dac_controller& operator=(const dac_controller& other) = delete;

    dac_controller& operator=(dac_controller&& other) = delete;

    void accept_frequencies(const float frequencies[], size_t size)
    {
        static_assert(DAC_DOUBLE_BUFFER_SIZE < std::numeric_limits<float>::max());

        //Populate buffer with sum of sine waves, mapped from [0, 1] => [0, 255]
        //Apply moving average filter
        for (size_t i = 0; i < DAC_DOUBLE_BUFFER_SIZE; i++)
        {
            //Current time is the ratio through the buffer, scaled to the amount of time that the buffer corresponds to
            const auto t = (static_cast<float>(i) / DAC_DOUBLE_BUFFER_SIZE) * DAC_DOUBLE_BUFFER_TIME_SECONDS;

            float sum = 0.f;
            size_t freq_used = 0;

            for (size_t f = 0; f < size; f++)
            {
                if (const auto freq = frequencies[f]; freq > 0)
                {
                    sum += (1.f + sinf(2.f * static_cast<float>(M_PI) * static_cast<float>(t) * freq)) / 2.f;
                    freq_used++;
                }
            }

            uint8_t next_value;

            //If no valid frequency data, set value to 0
            //If for some reason input length exceeds max float, also return 0, to prevent narrowing cast in else branch
            if (freq_used == 0 || freq_used > static_cast<size_t>(std::round(std::numeric_limits<float>::max())))
            {
                next_value = 0;
            }
            else
            {
                const auto scaled = static_cast<uint8_t>(std::clamp(static_cast<int>(std::round(0xFF * sum / static_cast<float>(freq_used))), 0, 0xFF));
                next_value = scaled;
            }

            //Apply moving average filter to smooth data. Treat the active buffer as coming before i = 0
            //for the purposes of this in order to smooth between buffers as well.
            //If near the start of the buffer, use a larger moving average window to help prevent large
            //breaks between buffers
            int moving_average_sum = next_value;
            int moving_average_count = 1;

            const auto moving_average_len = i > MOVING_AVERAGE_LEN
                ? MOVING_AVERAGE_LEN
                : MOVING_AVERAGE_LEN_BETWEEN_BUFFERS;

            for (size_t j = moving_average_len; j > 0; j--)
            {
                if (j <= i) //Within this buffer
                {
                    moving_average_sum += working_buffer[i - j];
                    moving_average_count++;
                }
                else //From end of active buffer
                {
                    if (const auto ndx_from_end = j - i;
                        ndx_from_end <= DAC_DOUBLE_BUFFER_SIZE) //Must not go past start of previous buffer
                    {
                        moving_average_sum += active_buffer[DAC_DOUBLE_BUFFER_SIZE - ndx_from_end];
                        moving_average_count++;
                    }
                }
            }

            working_buffer[i] = static_cast<uint8_t>(std::clamp(static_cast<int>(std::round((1. * moving_average_sum) / moving_average_count)), 0, 0xFF));
        }

        //Swap buffers and write to DAC
        swap_buffers();

        if (const auto dac_write_result = dac_continuous_write_cyclically(
            handle,
            active_buffer,
            DAC_DOUBLE_BUFFER_SIZE,
            nullptr
        ); dac_write_result != ESP_OK)
        {
            loge(NAMEOF(dac_controller), std::format("{} Failed to write to DAC. [0x{:04X}] {}",
                LOG_KEY, dac_write_result, esp_err_to_name(dac_write_result)));
        }
    }

    [[nodiscard]] static std::unique_ptr<dac_controller> try_create(const dac_channel_mask_t channels = DAC_CHANNEL_MASK_ALL, const dac_continuous_channel_mode_t channel_mode = DAC_CHANNEL_MODE_SIMUL)
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

        if (const auto enable_result = dac_continuous_enable(handle);
            enable_result != ESP_OK)
        {
            loge(NAMEOF(dac_controller), std::format("{} Failed to enable DAC channels. [0x{:04X}] {}",
                LOG_KEY, enable_result, esp_err_to_name(enable_result)));

            return nullptr;
        }

        return std::make_unique<dac_controller>(handle);
    }

    ~dac_controller()
    {
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

        delete[] active_buffer;
        delete[] working_buffer;
    }
};

#endif //AUDIO_CONTROLLER_DAC_CONTROLLER_H
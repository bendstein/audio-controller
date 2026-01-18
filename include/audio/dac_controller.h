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
    static constexpr size_t DAC_CFG_BUFFER_SIZE = 1 << 10;
    static constexpr int DAC_CFG_SAMPLE_RATE = 1 << 17;
    static constexpr int DAC_CFG_OFFSET = 0;

    static constexpr size_t DAC_DOUBLE_BUFFER_SIZE = 1 << 12;
    static constexpr float DAC_DOUBLE_BUFFER_TIME_SECONDS = static_cast<float>(DAC_DOUBLE_BUFFER_SIZE) / DAC_CFG_SAMPLE_RATE;
    static constexpr size_t MOVING_AVERAGE_LEN = 3;

    static constexpr size_t DAC_DOUBLE_BUFFER_SIZE_ADJUSTMENT = 1 << 8;
    static constexpr size_t DAC_DOUBLE_BUFFER_SIZE_TRUNCATED = DAC_DOUBLE_BUFFER_SIZE - DAC_DOUBLE_BUFFER_SIZE_ADJUSTMENT;

    dac_continuous_handle_t handle;
    size_t active_buffer_len = 0;
    uint8_t* active_buffer;
    uint8_t* working_buffer;

    size_t swap_buffers(const size_t working_buffer_len)
    {
        std::swap(active_buffer, working_buffer);
        const auto current = active_buffer_len;
        active_buffer_len = working_buffer_len;
        return current;
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

    void accept_frequencies(const float frequencies[], const size_t size)
    {
        static_assert(DAC_DOUBLE_BUFFER_SIZE < std::numeric_limits<float>::max());
        assert(size < std::numeric_limits<float>::max());

        //Special case if no frequencies are given: stop
        if (size == 0)
        {
            active_buffer_len = 0;
            if (const auto disable_result = dac_continuous_disable(handle);
                disable_result != ESP_OK)
            {
                FLOGE("{} Failed to disable DAC output. (0x{:02X}) {}", LOG_KEY, disable_result, esp_err_to_name(disable_result));
            }

            return;
        }

        for (size_t i = 0; i < DAC_DOUBLE_BUFFER_SIZE; i++)
        {
            //Current time is the ratio through the buffer, scaled to the amount of time that the buffer corresponds to
            const auto t = (static_cast<float>(i) / DAC_DOUBLE_BUFFER_SIZE) * DAC_DOUBLE_BUFFER_TIME_SECONDS;

            //Sum up sine waves, skipping 0s
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

            //If no frequency data, use 0, otherwise, average sine waves and scale to byte
            const auto next_value = freq_used == 0
                ? 0
                : static_cast<uint8_t>(std::clamp(static_cast<int>(std::round(0xFF * sum / static_cast<float>(freq_used))), 0, 0xFF));

            //Apply moving average filter to smooth data
            int moving_average_sum = next_value;
            int moving_average_count = 1;

            for (size_t j = MOVING_AVERAGE_LEN; j > 0; j--)
            {
                if (j <= i) //Within this buffer
                {
                    moving_average_sum += working_buffer[i - j];
                    moving_average_count++;
                }
                else if (active_buffer_len > 0) //From end of active buffer
                {
                    if (const auto ndx_from_end = j - i;
                        ndx_from_end <= active_buffer_len) //Must not go past start of previous buffer
                    {
                        moving_average_sum += active_buffer[active_buffer_len - ndx_from_end];
                        moving_average_count++;
                    }
                }
            }

            working_buffer[i] = static_cast<uint8_t>(std::clamp(static_cast<int>(std::round((1. * moving_average_sum) / moving_average_count)), 0, 0xFF));
        }

        //Truncate the end of the buffer for continuity when wrapping from end to start.
        //Subtracting DAC_LENGTH_ADJUSTMENT because for some reason the last 256 bits are
        //ignored.
        //Try to find the point closest to the end of the buffer which passes through 0xFF/2
        //with a positive slope
        //Sample interval is chosen arbitrarily. It's only purpose is to prevent potential
        //rounding issues from choosing points too close to each other
        constexpr size_t sample_interval = 4;
        constexpr uint8_t midpoint = 0xFF / 2;

        size_t working_buffer_len = DAC_DOUBLE_BUFFER_SIZE_TRUNCATED;

        uint8_t previous_value = 0;
        for (size_t i = DAC_DOUBLE_BUFFER_SIZE_TRUNCATED - 1; i >= sample_interval; i -= sample_interval)
        {
            const auto current_value = working_buffer[i];

            //Check that points are on opposite side of the midpoint,
            //and that this point is less than previous (moving backwards,
            //so this indicates ascending)
            if (current_value <= midpoint
                && previous_value >= midpoint
                && current_value < previous_value
            )
            {
                working_buffer_len = i;
                break;
            }

            previous_value = current_value;
        }

        // //Find the value closest to the end that is within epsilon of the
        // //first value, and use that as the end of the buffer, to help
        // //with continuity on wrap.
        // //For some reason, the dac eats the last 256 bytes, so account for
        // //that here as well
        // const auto first_value = working_buffer[0];
        // size_t working_buffer_len = DAC_DOUBLE_BUFFER_SIZE - DAC_LENGTH_ADJUSTMENT;
        // for (size_t i = DAC_DOUBLE_BUFFER_SIZE - DAC_LENGTH_ADJUSTMENT - 1; i > 0; i--)
        // {
        //     const auto current = working_buffer[i];
        //     if (const auto diff = first_value > current? (first_value - current) : (current - first_value);
        //         diff <= DAC_ZERO_EPS)
        //     {
        //         working_buffer_len = i + 1;
        //         break;
        //     }
        // }
        //
        // // working_buffer_len = DAC_DOUBLE_BUFFER_SIZE;

        //Swap buffers and write to DAC
        const auto prev_buffer_len = swap_buffers(working_buffer_len);

        FVERBOSE("First value: 0x{:02X}, Len: {}, Last Value: 0x{:02X}, True Last Value: 0x{:02X}",
            first_value,
            active_buffer_len,
            active_buffer[active_buffer_len - 1],
            active_buffer[DAC_DOUBLE_BUFFER_SIZE - 1]);

        //Re-enable DAC output if necessary
        if (prev_buffer_len == 0)
        {
            if (const auto enable_result = dac_continuous_enable(handle);
                enable_result != ESP_OK)
            {
                FLOGE("{} Failed to enable DAC output. (0x{:02X}) {}", LOG_KEY, enable_result, esp_err_to_name(enable_result));
            }
        }

        if (const auto dac_write_result = dac_continuous_write_cyclically(
            handle,
            active_buffer,
            active_buffer_len,
            nullptr
        ); dac_write_result != ESP_OK)
        {
            FLOGE("{} Failed to write to DAC. [0x{:04X}] {}",
                LOG_KEY, dac_write_result, esp_err_to_name(dac_write_result));
        }
    }

    [[nodiscard]] static std::unique_ptr<dac_controller> try_create(const dac_channel_mask_t channels = DAC_CHANNEL_MASK_ALL, const dac_continuous_channel_mode_t channel_mode = DAC_CHANNEL_MODE_SIMUL)
    {
        FLOGI("{} Creating DAC controller.", LOG_KEY);

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
            FLOGE("{} Failed to allocate DAC channels. [0x{:04X}] {}",
                LOG_KEY, alloc_result, esp_err_to_name(alloc_result));

            return nullptr;
        }

        // if (const auto enable_result = dac_continuous_enable(handle);
        //     enable_result != ESP_OK)
        // {
        //     FLOGE("{} Failed to enable DAC channels. [0x{:04X}] {}",
        //         LOG_KEY, enable_result, esp_err_to_name(enable_result));
        //
        //     return nullptr;
        // }

        return std::make_unique<dac_controller>(handle);
    }

    ~dac_controller()
    {
        //Dealloc DAC channel handle, associated task handle on destroy
        if (handle != nullptr)
        {
            try
            {
                FLOGI("{} Removing DAC channel handle 0x{:08X}.",
                     LOG_KEY,
                     reinterpret_cast<uintptr_t>(handle));

                if (const auto remove_dac_channel_result = dac_continuous_del_channels(handle);
                    remove_dac_channel_result != ESP_OK)
                {
                    FLOGE("{} Failed to remove DAC channel handle 0x{:08X}. [0x{:04X}] {}",
                        LOG_KEY,
                        reinterpret_cast<uintptr_t>(handle),
                        remove_dac_channel_result,
                        esp_err_to_name(remove_dac_channel_result));
                }
            }
            catch (std::exception& e)
            {
                FLOGE("{} An exception occurred while removing DAC channel handle 0x{:08X}: {}",
                    LOG_KEY, reinterpret_cast<uintptr_t>(handle), e.what());
            }
        }

        delete[] active_buffer;
        delete[] working_buffer;
    }
};

#endif //AUDIO_CONTROLLER_DAC_CONTROLLER_H
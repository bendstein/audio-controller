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
#include "rtos_utils.h"
#include "i2c/mcp4725.h"

enum struct dac_controller_start_result
{
    OK,
    ERR,
    ALREADY_STARTED
};

class dac_controller
{
public:
    static constexpr auto LOG_KEY = "[DAC Ctrl]";
private:
    static constexpr auto TASK_NAME = "[DAC Ctrl]";
    static constexpr auto WB_TASK_NAME = "[DAC Ctrl WB]";
    static constexpr auto TASK_PRIORITY = 2;
    static constexpr auto TASK_STACK_SIZE = 0x1000;
    static constexpr auto WB_TASK_PRIORITY = 1;
    static constexpr auto WB_TASK_STACK_SIZE = 0x1000;

    static constexpr auto TASK_MESSAGE_INDEX_WB_READY = 1;
    static constexpr auto WB_TASK_MESSAGE_INDEX_READY = 1;

    static constexpr int DAC_CFG_DESC_NUM = 10;
    static constexpr size_t DAC_CFG_BUFFER_SIZE = 1 << 11;
    static constexpr int DAC_CFG_SAMPLE_RATE = sine_tables::SAMPLE_RATE;
    static constexpr int DAC_CFG_OFFSET = 0;

    static constexpr size_t DAC_DOUBLE_BUFFER_SIZE = 1 << 14;
    static constexpr float DAC_DOUBLE_BUFFER_TIME_SECONDS = static_cast<float>(DAC_DOUBLE_BUFFER_SIZE) / DAC_CFG_SAMPLE_RATE;
    static constexpr size_t MOVING_AVERAGE_LEN = 4;

    static constexpr size_t FREQ_BUFFER_CAPACITY = 32;
    static constexpr size_t DAC_WRITE_QUEUE_CAPACITY = 8;

    dac_continuous_handle_t handle;
    TaskHandle_t task;
    TaskHandle_t wb_task;

    size_t active_buffer_len = 0;
    uint8_t* active_buffer;
    uint8_t* working_buffer;

    std::atomic_bool tone_data_changed { true };
    size_t tone_data_len = 0;
    musical_note_tone* tone_data;

    SemaphoreHandle_t tones_mux;
    SemaphoreHandle_t swap_buffers_mux;
    SemaphoreHandle_t task_started_sem;
    QueueHandle_t dac_write_queue_handle;

    void swap_buffers(size_t& working_buffer_len)
    {
        FVERBOSE("{} Swap buffers.", LOG_KEY);

        rtos_semaphore_handle lock { swap_buffers_mux };

        //In case that working buffer is empty, fill it with 0
        //and set length to max
        if (working_buffer_len == 0)
        {
            FVERBOSE("{} No data in working buffer. Zeroing buffer and setting length to max.", LOG_KEY);
            memset(working_buffer, 0, DAC_DOUBLE_BUFFER_SIZE);
            working_buffer_len = DAC_DOUBLE_BUFFER_SIZE;
        }

        //Otherwise, swap active and working buffers
        std::swap(active_buffer, working_buffer);
        active_buffer_len = working_buffer_len;
    }

    static bool IRAM_ATTR dac_isr(dac_continuous_handle_t, const dac_event_data_t* ev, void* isr_param)
    {
        //ISR adds event to the queue, does nothing else
        BaseType_t should_wake = pdFALSE;

        if (isr_param == nullptr)
            return should_wake;

        const auto queue = static_cast<QueueHandle_t>(isr_param);

        if (xQueueIsQueueFullFromISR(queue)) //Dequeue oldest item on full
        {
            dac_event_data_t dummy;
            xQueueReceiveFromISR(queue, &dummy, &should_wake);
        }

        //Enqueue this event
        xQueueSendFromISR(queue, ev, &should_wake);

        return should_wake;
    }

    [[noreturn]]
    static void calculate_working_buffer_task(void* task_param)
    {
        assert(task_param != nullptr);

        try
        {
            if (const auto self = static_cast<dac_controller*>(task_param);
                self != nullptr)
            {
                //Offset for working buffer calculation, for continuity between adjacent calculations.
                //Reset whenever the frequency buffer is changed.
                size_t offset = 0;

                size_t working_buffer_len = 0;

                bool first = true;

                FLOGI("{} Task {} starting main loop.", LOG_KEY, WB_TASK_NAME);

                do
                {
                    FVERBOSE("{} Task {} wait for frequency data.", LOG_KEY, WB_TASK_NAME);

                    //Wait for frequencies to be available to create buffer data from
                    if (const auto maybe_frequencies_handle = rtos_semaphore_handle::try_take(self->tones_mux);
                        maybe_frequencies_handle != nullptr)
                    {
                        if (self->tone_data_changed) //Reset offset on frequency change
                        {
                            self->tone_data_changed = false;
                            offset = 0;
                        }

                        working_buffer_len = self->populate_working_buffer(offset);

                        offset += working_buffer_len;
                    }
                    else
                    {
                        FLOGE("{} Didn't successfully acquire mutex for frequency data in task {}.", LOG_KEY, WB_TASK_NAME);
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                        continue;
                    }

                    //On first time, await main task start
                    if (first)
                    {
                        do
                        {
                            FLOGI("{} Task {} waiting for task {} to start.", LOG_KEY, WB_TASK_NAME, TASK_NAME);

                            if (const auto take_result = xSemaphoreTake(self->task_started_sem, portMAX_DELAY);
                                take_result == pdPASS)
                            { }
                            else
                            {
                                FLOGE("{} Task {} didn't successfully acquire semaphore indicating that task {} has started.", LOG_KEY, WB_TASK_NAME, TASK_NAME);
                                vTaskDelay(10 / portTICK_PERIOD_MS);
                                continue;
                            }

                            break;
                        }
                        while (true);

                        first = false;
                    }

                    //Indicate that working buffer is ready for swap, including its length
                    FVERBOSE("{} Task {} sending message {} to task {}:{}.", LOG_KEY, WB_TASK_NAME, working_buffer_len, TASK_NAME, TASK_MESSAGE_INDEX_WB_READY);
                    xTaskNotifyIndexed(self->task, TASK_MESSAGE_INDEX_WB_READY, working_buffer_len, eSetValueWithOverwrite);

                    vTaskDelay(10 / portTICK_PERIOD_MS);

                    //Wait for message to recalculate working buffer
                    FVERBOSE("{} Task {} waiting for message to recalculate (ndx {}).", LOG_KEY, WB_TASK_NAME, WB_TASK_MESSAGE_INDEX_READY);

                    //Only wait for a set amount of time before recalculating anyways
                    xTaskNotifyWaitIndexed(WB_TASK_MESSAGE_INDEX_READY, 0xFFFFFFFF, 0xFFFFFFFF,
                            nullptr, 100 / portTICK_PERIOD_MS);
                }
                while (true);
            }

            FLOGE("{} Invalid task param for task {}", LOG_KEY, WB_TASK_NAME);
        }
        catch (std::exception& e)
        {
            FLOGE("{} An exception occurred while starting task {}: {}", LOG_KEY, WB_TASK_NAME, e.what());
        }

        do { vTaskDelay(portMAX_DELAY); } while (true);
    }

    [[noreturn]]
    static void dac_task(void* task_param)
    {
        assert(task_param != nullptr);

        try
        {
            if (const auto self = static_cast<dac_controller*>(task_param);
                self != nullptr)
            {
                //Flag that task has started
                xSemaphoreGive(self->task_started_sem);

                size_t working_buffer_len;

                FLOGI("{} Task {} waiting for initial buffer calculation.", LOG_KEY, TASK_NAME);

                //Wait for initial buffer calculation, updating working buffer length
                do
                {
                    uint32_t working_buffer_len_temp;

                    if (const auto wait_result = xTaskNotifyWaitIndexed(TASK_MESSAGE_INDEX_WB_READY, 0xFFFFFFFF, 0xFFFFFFFF,
                        &working_buffer_len_temp, portMAX_DELAY);
                        wait_result != pdPASS)
                    {
                        FLOGE("{} Didn't successfully receive initial buffer calculation message in task {}.", LOG_KEY, WB_TASK_NAME);
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                        continue;
                    }

                    working_buffer_len = static_cast<size_t>(working_buffer_len_temp);

                    break;
                }
                while (true);

                //Start write
                do
                {
                    FLOGI("{} Task {} starting DAC continuous write.", LOG_KEY, TASK_NAME);

                    if (const auto start_write_result = dac_continuous_start_async_writing(self->handle);
                        start_write_result != ESP_OK)
                    {
                        FLOGE("{} Task {} failed to start DAC async write. [0x{:04X}] {}", LOG_KEY, TASK_NAME, start_write_result, esp_err_to_name(start_write_result));
                        vTaskDelay(50 / portTICK_PERIOD_MS);
                        continue;
                    }

                    break;
                }
                while (true);

                FLOGI("{} Task {} starting main loop.", LOG_KEY, TASK_NAME);

                do
                {
                    //Swap buffers. In specific case where working buffer len is 0, will be updated to max length
                    //and buffer will be zeroed
                    self->swap_buffers(working_buffer_len);

                    //Notify working buffer task that it can start recalculating
                    if (self->wb_task != nullptr)
                    {
                        FVERBOSE("{} Task {} sending message to task {}:{} indicating that buffer should be recalculated.",
                            LOG_KEY, TASK_NAME, WB_TASK_NAME, WB_TASK_MESSAGE_INDEX_READY);

                        xTaskNotifyIndexed(self->wb_task, WB_TASK_MESSAGE_INDEX_READY, 1, eSetValueWithoutOverwrite);
                    }

                    dac_event_data_t ev;
                    size_t cursor = 0;

                    if (self->active_buffer_len > 0)
                        FVERBOSE("{} Task {} writing {} bytes to DAC.", LOG_KEY, TASK_NAME, self->active_buffer_len);

                    //Write active buffer to DAC DMA buffer
                    while (cursor < self->active_buffer_len)
                    {
                        //Wait for interrupt to enqueue event
                        FVERBOSE("{} Task {} waiting for next DAC event.", LOG_KEY, TASK_NAME);

                        if (self->tone_data_changed) //If frequencies changed, recalculate buffer
                            break;

                        //Wait for next event, continuing to next iteration if takes too long
                        if (!xQueueReceive(self->dac_write_queue_handle, &ev, 5 / portTICK_PERIOD_MS))
                            continue;

                        if (self->tone_data_changed) //If frequencies changed, recalculate buffer
                            break;

                        size_t loaded = 0;

                        if (const auto write_result = dac_continuous_write_asynchronously(self->handle,
                            static_cast<uint8_t*>(ev.buf), ev.buf_size,
                            self->active_buffer + cursor, self->active_buffer_len - cursor, &loaded);
                            write_result != ESP_OK)
                        {
                            FLOGE("{} Didn't successfully write to DAC DMA buffer in task {}. [0x{:02}] {}", LOG_KEY, TASK_NAME, write_result, esp_err_to_name(write_result));
                        }
                        else
                        {
                            cursor += loaded;
                        }

                        // FVERBOSE("{} Task {} wrote {} bytes to DAC.", LOG_KEY, TASK_NAME, loaded);
                    }

                    //Swap in working buffer
                    do
                    {
                        FVERBOSE("{} Task {} waiting for working buffer to be ready (ndx: {})", LOG_KEY, TASK_NAME, TASK_MESSAGE_INDEX_WB_READY);

                        uint32_t working_buffer_len_temp;

                        if (const auto wait_result = xTaskNotifyWaitIndexed(TASK_MESSAGE_INDEX_WB_READY, 0xFFFFFFFF, 0xFFFFFFFF,
                            &working_buffer_len_temp, portMAX_DELAY);
                            wait_result != pdPASS)
                        {
                            FLOGE("{} Didn't successfully receive buffer calculation message in task {}.", LOG_KEY, WB_TASK_NAME);
                            vTaskDelay(10 / portTICK_PERIOD_MS);
                            continue;
                        }

                        working_buffer_len = static_cast<size_t>(working_buffer_len_temp);

                        break;
                    }
                    while (true);

                    FVERBOSE("{} Task {} received working buffer w/ length {}", LOG_KEY, TASK_NAME, working_buffer_len);
                }
                while (true);
            }

            FLOGE("{} Invalid task param for task {}", LOG_KEY, TASK_NAME);
        }
        catch (std::exception& e)
        {
            FLOGE("{} An exception occurred while starting task {}: {}", LOG_KEY, TASK_NAME, e.what());
        }

        do { vTaskDelay(portMAX_DELAY); } while (true);
    }

    [[nodiscard]] size_t populate_working_buffer(const size_t offset) const
    {
        FVERBOSE("{} Populate working buffer starting at offset {}", LOG_KEY, offset);

        static_assert(DAC_DOUBLE_BUFFER_SIZE < std::numeric_limits<float>::max());
        static_assert(FREQ_BUFFER_CAPACITY < std::numeric_limits<float>::max());
        assert(handle != nullptr);
        assert(tone_data_len < FREQ_BUFFER_CAPACITY);

        if (tone_data_len == 0) //No frequency data given
            return 0;

        for (size_t i = 0; i < DAC_DOUBLE_BUFFER_SIZE; i++)
        {
            //Use data from pre-calculated sine wave tables, average together
            uint32_t sum = 0;
            size_t tones_used = 0;

            for (auto f = 0; f < tone_data_len; f++)
            {
                if (const auto& tone = tone_data[f]; !tone.is_invalid())
                {
                    sum += tone.sine_table()->get_value(i + offset);
                    tones_used++;
                }
            }

            const auto next_value = tones_used == 0
                ? 0
                : sum / tones_used;

            //Apply moving average filter to smooth data
            int moving_average_sum = static_cast<int>(next_value);
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

        // return working_buffer_len;

        return DAC_DOUBLE_BUFFER_SIZE;
    }
public:
    dac_controller() : handle(nullptr), task(nullptr), wb_task(nullptr)
    {
        tones_mux = xSemaphoreCreateMutex();
        swap_buffers_mux = xSemaphoreCreateMutex();
        task_started_sem = xSemaphoreCreateBinary();
        dac_write_queue_handle = xQueueCreate(DAC_WRITE_QUEUE_CAPACITY, sizeof(dac_event_data_t));

        active_buffer = new uint8_t[DAC_DOUBLE_BUFFER_SIZE] {};
        working_buffer = new uint8_t[DAC_DOUBLE_BUFFER_SIZE] {};
        tone_data = new musical_note_tone[FREQ_BUFFER_CAPACITY] {};
    }

    dac_controller(const dac_controller& other) = delete;
    dac_controller(dac_controller&& other) = delete;
    dac_controller& operator=(const dac_controller& other) = delete;
    dac_controller& operator=(dac_controller&& other) = delete;

    dac_controller_start_result start()
    {
        try
        {
            if (task != nullptr && wb_task != nullptr)
            {
                FLOGW("{} Tasks {}, {} are already started ({}: 0x{:08X}, {}: 0x{:08X})",
                    LOG_KEY,
                    WB_TASK_NAME, TASK_NAME,
                    WB_TASK_NAME, reinterpret_cast<uintptr_t>(wb_task),
                    TASK_NAME, reinterpret_cast<uintptr_t>(task));

                return dac_controller_start_result::ALREADY_STARTED;
            }

            if (wb_task == nullptr)
            {
                if (const auto wb_result = xTaskCreate(
                    calculate_working_buffer_task,
                    WB_TASK_NAME,
                    WB_TASK_STACK_SIZE,
                    this,
                    WB_TASK_PRIORITY,
                    &this->wb_task
                ); wb_result != pdPASS)
                {
                    FLOGE("{} Failed to start task {}: 0x{:04X}", LOG_KEY, WB_TASK_NAME, static_cast<uint16_t>(wb_result));
                    return dac_controller_start_result::ERR;
                }
            }

            FLOGI("{} Task {} is started (0x{:08X}).", LOG_KEY, WB_TASK_NAME, reinterpret_cast<uintptr_t>(wb_task));

            if (task == nullptr)
            {
                if (const auto result = xTaskCreate(
                    dac_task,
                    TASK_NAME,
                    TASK_STACK_SIZE,
                    this,
                    TASK_PRIORITY,
                    &this->task
                ); result != pdPASS)
                {
                    FLOGE("{} Failed to start task {}: 0x{:04X}", LOG_KEY, TASK_NAME, static_cast<uint16_t>(result));
                    return dac_controller_start_result::ERR;
                }
            }

            FLOGI("{} Task {} is started (0x{:08X}).", LOG_KEY, TASK_NAME, reinterpret_cast<uintptr_t>(task));

            return dac_controller_start_result::OK;
        }
        catch (const std::exception& e)
        {
            FLOGE("{} An exception occurred while starting task: {}", LOG_KEY, e.what());
        }

        return dac_controller_start_result::ERR;
    }

    void write(const musical_note_tone data[], const size_t data_len)
    {
        assert(data_len <= FREQ_BUFFER_CAPACITY);

        FVERBOSE("{} Waiting to write {} frequencies to DAC controller.", LOG_KEY, data_len);

        //Wait to be able to record frequency data
        if (const auto maybe_frequencies_handle = rtos_semaphore_handle::try_take(tones_mux);
            maybe_frequencies_handle != nullptr)
        {
            tone_data_changed = false;

            for (size_t i = 0; i < data_len; i++)
            {
                if (!tone_data[i].is_equivalent(data[i]))
                {
                    tone_data_changed = true;
                    tone_data[i] = data[i];
                }
            }

            if (tone_data_len != data_len)
            {
                FVERBOSE("{} Frequency data was updated.", LOG_KEY);
                tone_data_changed = true;
                tone_data_len = data_len;

                //If changed, send message to update working buffer
                if (wb_task != nullptr)
                {
                    FVERBOSE("{} Sending message to task {}:{} indicating that buffer should be recalculated.",
                        LOG_KEY, WB_TASK_NAME, WB_TASK_MESSAGE_INDEX_READY);

                    xTaskNotifyIndexed(wb_task, WB_TASK_MESSAGE_INDEX_READY, 1, eSetValueWithoutOverwrite);
                }
            }
            else
            {
                FVERBOSE("{} No frequencies were changed.", LOG_KEY);
            }
        }
        else
        {
            FLOGE("{} Didn't successfully acquire mutex to update frequency data.", LOG_KEY);
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

        auto ctrl = std::make_unique<dac_controller>();
        ctrl->handle = handle;

        if (const auto enable_result = dac_continuous_enable(handle);
            enable_result != ESP_OK)
        {
            FLOGE("{} Failed to enable DAC continuous write. [0x{:04X}] {}", LOG_KEY, enable_result, esp_err_to_name(enable_result));
            return nullptr;
        }

        constexpr auto cb = dac_event_callbacks_t {
            .on_convert_done = dac_isr,
            .on_stop = nullptr
        };

        if (const auto register_cb_result = dac_continuous_register_event_callback(handle, &cb, ctrl->dac_write_queue_handle);
            register_cb_result != ESP_OK)
        {
            FLOGE("{} Failed to register DAC callbacks. [0x{:04X}] {}", LOG_KEY, register_cb_result, esp_err_to_name(register_cb_result));
            return nullptr;
        }

        return ctrl;
    }

    ~dac_controller()
    {
        //Dealloc DAC channel handle, associated task handles on destroy
        if (task != nullptr)
        {
            try
            {
                FLOGI("{} Stopping task {}/0x{:08X}.",
                     LOG_KEY,
                     WB_TASK_NAME,
                     reinterpret_cast<uintptr_t>(wb_task));

                vTaskDelete(wb_task);
            }
            catch (std::exception& e)
            {
                FLOGE("{} An exception occurred while stopping task 0x{:08X}: {}",
                    LOG_KEY, reinterpret_cast<uintptr_t>(task), e.what());
            }
        }

        if (task != nullptr)
        {
            try
            {
                FLOGI("{} Stopping task {}/0x{:08X}.",
                     LOG_KEY,
                     TASK_NAME,
                     reinterpret_cast<uintptr_t>(task));

                vTaskDelete(task);
            }
            catch (std::exception& e)
            {
                FLOGE("{} An exception occurred while stopping task 0x{:08X}: {}",
                    LOG_KEY, reinterpret_cast<uintptr_t>(task), e.what());
            }
        }

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

        delete[] tone_data;
        delete[] active_buffer;
        delete[] working_buffer;
    }
};

#endif //AUDIO_CONTROLLER_DAC_CONTROLLER_H
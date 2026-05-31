//
// Created by bendstein on 1/13/2026.
//

#ifndef AUDIO_CONTROLLER_DAC_CONTROLLER_H
#define AUDIO_CONTROLLER_DAC_CONTROLLER_H

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
#include "fixed_vec.h"
#include "tones.h"

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
    static constexpr size_t TONE_DATA_CAPACITY = 32;
private:
    static constexpr auto TASK_NAME = "<DAC Ctrl>";
    static constexpr auto TASK_PRIORITY = 2;
    static constexpr auto TASK_STACK_SIZE = 0x1000;
    static constexpr size_t TASK_MESSAGE_QUEUE_INDEX_TONE_CHANGE = 0;

    /**
     * Count of DMA descriptors for DAC config,
     * should be at least 5
     */
    static constexpr int DAC_CFG_DESC_NUM = 10;

    /**
     * DMA buffer size for DAC, should be 32-4092, multiple of 4
     */
    static constexpr size_t DAC_CFG_BUFFER_SIZE = 1 << 11;

    /**
     * Sample rate in Hz for DAC
     */
    static constexpr int DAC_CFG_SAMPLE_RATE = musical_note_data::SAMPLE_RATE;

    /**
     * Offset of DAC data, -128 to 127
     */
    static constexpr int DAC_CFG_OFFSET = 0;

    /**
     * Moving average as filter when populating DAC buffer
     */
    static constexpr size_t MOVING_AVERAGE_LEN = 1;

    /**
     * Buffer where the values for the given tones
     * are summed, later transferred to DAC buffer
     */
    static constexpr size_t BUFFER_LEN = 1 << 10;

    /**
     * Max number of events in the event queue that is populated in ISR,
     * oldest item is dropped once exceeded
     */
    static constexpr size_t DAC_WRITE_QUEUE_CAPACITY = 8;

    dac_continuous_handle_t handle;
    TaskHandle_t task;

    uint8_t* buffer;

    fixed_vec<musical_note_tone, TONE_DATA_CAPACITY> tone_data {};

    SemaphoreHandle_t tones_mux;
    SemaphoreHandle_t task_started_sem;
    QueueHandle_t dac_write_queue_handle;

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
    static void dac_task(void* task_param)
    {
        assert(task_param != nullptr);

        try
        {
            if (const auto self = static_cast<dac_controller*>(task_param);
                self != nullptr)
            {
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

                fixed_vec<musical_note_tone, TONE_DATA_CAPACITY> local_tone_data {};

                dac_event_data_t ev;
                size_t offset = 0; //Offset for calculating data buffer
                // size_t cursor = 0; //Offset for writing to DAC buffer

                //Read initial tone data
                FVERBOSE("{} Task {} waiting to read initial tone data to DAC controller.", LOG_KEY, TASK_NAME);

                do
                {
                    if (const auto maybe_tones_handle = rtos_semaphore_handle::try_take(self->tones_mux, portMAX_DELAY);
                        maybe_tones_handle != nullptr)
                    {
                        FVERBOSE("{} Task {} next chord: [{}]", LOG_KEY, TASK_NAME, self->tone_data.to_string([](const musical_note_tone& t) -> std::string { return t.name(); }));
                        // local_tone_data = std::move(self->tone_data);
                        local_tone_data.clone_from(self->tone_data);

                        //Clear change notification if present (do not wait)
                        xTaskNotifyWaitIndexed(TASK_MESSAGE_QUEUE_INDEX_TONE_CHANGE, 0, 0, nullptr, 0);
                        break;
                    }

                    FLOGE("{} Didn't successfully acquire mutex to read new tone data. Will retry.", LOG_KEY);
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                }
                while (true);

                do
                {
                    //Load data at offset into calculated data buffer
                    FVERBOSE("{} Task {} loading audio data at {}.", LOG_KEY, TASK_NAME, offset);
                    self->populate_buffer(self->buffer, BUFFER_LEN, offset, local_tone_data);

                    //Recalculate data buffer when a change event is present while waiting for DAC event.
                    do
                    {
                        if (xTaskNotifyWaitIndexed(TASK_MESSAGE_QUEUE_INDEX_TONE_CHANGE, 0, 0, nullptr, 0) == pdTRUE)
                        {
                            //Try read in current tone data, but give up and keep going after a short timeout
                            if (const auto maybe_tones_handle = rtos_semaphore_handle::try_take(self->tones_mux, 50 / portTICK_PERIOD_MS);
                                maybe_tones_handle != nullptr)
                            {
                                //Clear change notification if present (do not wait)
                                xTaskNotifyWaitIndexed(TASK_MESSAGE_QUEUE_INDEX_TONE_CHANGE, 0, 0, nullptr, 0);

                                FVERBOSE("{} Task {} next chord: [{}]", LOG_KEY, TASK_NAME, self->tone_data.to_string([](const musical_note_tone& t) -> std::string { return t.name(); }));
                                local_tone_data.clone_from(self->tone_data);

                                //New set of data, reset offset
                                offset = 0;
                                FVERBOSE("{} Task {} loading updated audio data at {}.", LOG_KEY, TASK_NAME, offset);
                                self->populate_buffer(self->buffer, BUFFER_LEN, offset, local_tone_data);
                            }
                            else
                            {
                                FLOGE("{} Task {} didn't successfully acquire mutex to read new tone data.", LOG_KEY, TASK_NAME);
                            }
                        }
                    }
                    while (!xQueueReceive(self->dac_write_queue_handle, &ev, 100 / portTICK_PERIOD_MS));

                    FVERBOSE("{} Task {} received DAC event.", LOG_KEY, TASK_NAME);

                    size_t loaded = 0;

                    //Try write to DAC
                    if (const auto write_result = dac_continuous_write_asynchronously(self->handle,
                        static_cast<uint8_t*>(ev.buf), ev.buf_size,
                    // self->buffer + cursor, BUFFER_LEN - cursor, &loaded);
                    self->buffer, BUFFER_LEN, &loaded);
                        write_result != ESP_OK)
                    {
                        FLOGE("{} Didn't successfully write to DAC DMA buffer in task {}. [0x{:02}] {}", LOG_KEY, TASK_NAME, write_result, esp_err_to_name(write_result));
                    }
                    // else
                    // {
                    //     cursor += loaded;
                    // }

                    FVERBOSE("{} Task {} wrote {} bytes to DAC.", LOG_KEY, TASK_NAME, loaded);

                    //If data buffer length is greater than loaded, then not all data was loaded into DAC.
                    //Reduce offset so that data continues at correct offset.
                    if (BUFFER_LEN > loaded)
                        offset -= BUFFER_LEN - loaded;
                }
                while (true);

                /*
                do
                {
                    //Try read in current tone data, but give up and keep going after a short timeout
                    FVERBOSE("{} Task {} waiting to read tone data to DAC controller.", LOG_KEY, TASK_NAME);

                    if (const auto maybe_tones_handle = rtos_semaphore_handle::try_take(self->tones_mux, 50 / portTICK_PERIOD_MS);
                        maybe_tones_handle != nullptr)
                    {
                        FVERBOSE("{} Task {} next chord: [{}]", LOG_KEY, TASK_NAME, self->tone_data.to_string([](const musical_note_tone& t) -> std::string { return t.name(); }));
                        // local_tone_data = std::move(self->tone_data);
                        local_tone_data.clone_from(self->tone_data);

                        //Clear change notification if present (do not wait)
                        xTaskNotifyWaitIndexed(TASK_MESSAGE_QUEUE_INDEX_TONE_CHANGE, 0, 0, nullptr, 0);
                    }
                    else
                    {
                        FLOGE("{} Didn't successfully acquire mutex to read new tone data.", LOG_KEY);
                    }

                    FVERBOSE("{} Task {} loading buffer.", LOG_KEY, TASK_NAME);
                    self->populate_buffer(self->buffer, BUFFER_LEN, offset, local_tone_data);

                    size_t cursor = 0;
                    while (cursor < BUFFER_LEN)
                    {
                        FVERBOSE("{} Task {} waiting for next DAC event.", LOG_KEY, TASK_NAME);

                        //Check if there is a change notification. If so, recalculate. Continue doing this until a DAC event is received.
                        size_t next_buffer_offset;
                        do
                        {
                            next_buffer_offset = offset; //Don't want to actually increase offset until a dac event is received

                            if (xTaskNotifyWaitIndexed(TASK_MESSAGE_QUEUE_INDEX_TONE_CHANGE, 0, 0, nullptr, 0) == pdTRUE)
                            {
                                //Try read in current tone data, but give up and keep going after a short timeout
                                FVERBOSE("{} Task {} waiting to read tone data to DAC controller.", LOG_KEY, TASK_NAME);

                                if (const auto maybe_tones_handle = rtos_semaphore_handle::try_take(self->tones_mux, 50 / portTICK_PERIOD_MS);
                                    maybe_tones_handle != nullptr)
                                {
                                    FVERBOSE("{} Task {} next chord: [{}]", LOG_KEY, TASK_NAME, self->tone_data.to_string([](const musical_note_tone& t) -> std::string { return t.name(); }));

                                    // local_tone_data = std::move(self->tone_data);
                                    local_tone_data.clone_from(self->tone_data);

                                    //Clear change notification if present (do not wait)
                                    xTaskNotifyWaitIndexed(TASK_MESSAGE_QUEUE_INDEX_TONE_CHANGE, 0, 0, nullptr, 0);
                                }
                                else
                                {
                                    FLOGE("{} Task {} didn't successfully acquire mutex to read new tone data.", LOG_KEY, TASK_NAME);
                                }

                                FVERBOSE("{} Task {} loading buffer.", LOG_KEY, TASK_NAME);
                                self->populate_buffer(self->buffer, BUFFER_LEN, next_buffer_offset, local_tone_data);

                                //Reset cursor, as this is a new set of data
                                cursor = 0;
                            }
                        } while (!xQueueReceive(self->dac_write_queue_handle, &ev, 100 / portTICK_PERIOD_MS));

                        FVERBOSE("{} Task {} received DAC event.", LOG_KEY, TASK_NAME);

                        offset = next_buffer_offset;

                        size_t loaded = 0;
                        if (const auto write_result = dac_continuous_write_asynchronously(self->handle,
                            static_cast<uint8_t*>(ev.buf), ev.buf_size,
                            self->buffer + cursor, BUFFER_LEN - cursor, &loaded);
                            write_result != ESP_OK)
                        {
                            FLOGE("{} Didn't successfully write to DAC DMA buffer in task {}. [0x{:02}] {}", LOG_KEY, TASK_NAME, write_result, esp_err_to_name(write_result));
                        }
                        else
                        {
                            cursor += loaded;
                        }

                        FVERBOSE("{} Task {} wrote {} bytes to DAC.", LOG_KEY, TASK_NAME, loaded);
                    }

                }
                while (true);
                */
            }

            FLOGE("{} Invalid task param for task {}", LOG_KEY, TASK_NAME);
        }
        catch (std::exception& e)
        {
            FLOGE("{} An exception occurred while starting task {}: {}", LOG_KEY, TASK_NAME, e.what());
        }

        do { vTaskDelay(portMAX_DELAY); } while (true);
    }

    /**
     * @param buf data buffer to write into. Is mutable.
     * @param buf_len Length of data buffer. Not mutated.
     * @param offset Current offset int buf. Is mutated with new offset after write.
     * @param tones Tones to write. Not mutated.
     */
    void populate_buffer(uint8_t* buf, const size_t buf_len, size_t& offset, const fixed_vec<musical_note_tone, TONE_DATA_CAPACITY>& tones)
    {
        FVERBOSE("{} Populate working buffer starting at offset {}", LOG_KEY, offset);

        assert(handle != nullptr);

        if (tones.empty()) //No data given. Clear buffer and reset offset.
        {
            FVERBOSE("{} Clearing working buffer.", LOG_KEY, offset);
            memset(buf, 0, buf_len);
            offset = 0;
            return;
        }

        // FLOGI("Chord [{}]", tones[0].name());

        for (size_t i = 0; i < buf_len; i++)
        {
            //Use data from pre-calculated sine wave tables, average together
            uint32_t sum = 0;
            size_t tones_used = 0;

            for (auto f = 0; f < tones.size(); f++)
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
                //Loop around to end of buffer (by not more than offset) if moving average index would
                //be negative
                if (j <= i) //Within this buffer
                {
                    moving_average_sum += buf[i - j];
                }
                else
                {
                    if (const auto ndx_from_end = j - i;
                        ndx_from_end <= offset)
                    {
                        moving_average_sum += buf[buf_len - ndx_from_end];
                    }
                    else
                    {
                        break;
                    }
                }

                moving_average_count++;
            }

            buf[i] = static_cast<uint8_t>(std::clamp(static_cast<int>(std::round((1. * moving_average_sum) / moving_average_count)), 0, 0xFF));
        }

        offset += buf_len;
    }
public:
    dac_controller() : handle(nullptr), task(nullptr)
    {
        tones_mux = xSemaphoreCreateMutex();
        task_started_sem = xSemaphoreCreateBinary();
        dac_write_queue_handle = xQueueCreate(DAC_WRITE_QUEUE_CAPACITY, sizeof(dac_event_data_t));

        buffer = new uint8_t[BUFFER_LEN];
    }

    dac_controller(const dac_controller& other) = delete;
    dac_controller(dac_controller&& other) = delete;
    dac_controller& operator=(const dac_controller& other) = delete;
    dac_controller& operator=(dac_controller&& other) = delete;

    dac_controller_start_result start()
    {
        try
        {
            if (task != nullptr)
            {
                FLOGW("{} Task {} is already started (0x{:08X})", LOG_KEY, TASK_NAME, reinterpret_cast<uintptr_t>(task));
                return dac_controller_start_result::ALREADY_STARTED;
            }

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

            FLOGI("{} Task {} is started (0x{:08X}).", LOG_KEY, TASK_NAME, reinterpret_cast<uintptr_t>(task));

            return dac_controller_start_result::OK;
        }
        catch (const std::exception& e)
        {
            FLOGE("{} An exception occurred while starting task: {}", LOG_KEY, e.what());
        }

        return dac_controller_start_result::ERR;
    }

    void write(const fixed_vec<musical_note_tone, TONE_DATA_CAPACITY>& data)
    {
        FVERBOSE("{} Waiting to write {} tones ([{}]) to DAC controller.", LOG_KEY, data.size(),
            data.to_string([](const musical_note_tone& t) -> std::string { return t.name(); }));

        //Wait to be able to record tone data. If changed, then clone into and notify of change
        if (const auto maybe_tones_handle = rtos_semaphore_handle::try_take(tones_mux);
            maybe_tones_handle != nullptr)
        {
            if (const auto has_change = !data.sequence_equals(tone_data); has_change)
            {
                FVERBOSE("{} Writing updated tone data.", LOG_KEY);

                // data = std::move(tone_data);
                tone_data.clone_from(data);
                xTaskNotifyIndexed(task, TASK_MESSAGE_QUEUE_INDEX_TONE_CHANGE, true, eNoAction);

                FVERBOSE("{} New tone data: [{}].", LOG_KEY,
                    tone_data.to_string([](const musical_note_tone& t) -> std::string { return t.name(); }));
            }
            else
            {
                FVERBOSE("{} Tone data is unchanged.", LOG_KEY);
            }
        }
        else
        {
            FLOGE("{} Didn't successfully acquire mutex to update tone data.", LOG_KEY);
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

        delete[] buffer;
    }
};

#endif //AUDIO_CONTROLLER_DAC_CONTROLLER_H
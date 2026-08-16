//
// Created by bendstein on 8/15/2026.
//

#ifndef AUDIO_CONTROLLER_SIGNAL_GENERATOR_H
#define AUDIO_CONTROLLER_SIGNAL_GENERATOR_H

#include <format>
#include <memory>
#include <string>
#include <utility>
#include <driver/dac_continuous.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_common.h"
#include "rtos_utils.h"
#include "fixed_vec.h"
#include "tones.h"

enum struct signal_generator_start_result
{
    OK,
    ERR,
    ALREADY_STARTED
};

struct signal_generator_tone
{
    musical_note_tone_volume tone;
    uint32_t period;

    [[nodiscard]] bool is_zero() const { return period == 0 || tone.is_zero(); }
    [[nodiscard]] bool is_invalid() const { return tone.is_invalid(); }
    [[nodiscard]] bool is_zero_or_invalid() const { return period == 0 || tone.is_invalid(); }

    bool operator==(const signal_generator_tone& other) const { return tone == other.tone && period == other.period; }

    [[nodiscard]] static consteval signal_generator_tone create_zero() { return signal_generator_tone(musical_note_tone_volume::create_zero(), 0); }
    [[nodiscard]] static consteval signal_generator_tone create_invalid() { return signal_generator_tone(musical_note_tone_volume::create_invalid(), 0); }
};

class signal_generator
{
public:
    static constexpr auto LOG_KEY = "[Signal Gen]";
    static constexpr size_t TONE_DATA_CAPACITY = 8;

    typedef fixed_vec<musical_note_tone_volume, TONE_DATA_CAPACITY> tone_vec_t;
    typedef fixed_vec<signal_generator_tone, TONE_DATA_CAPACITY> tone_period_vec_t;
private:
    static constexpr auto TASK_NAME = "<Signal Gen>";
    static constexpr auto TASK_PRIORITY = 8;
    static constexpr auto TASK_STACK_SIZE = 0x1000;

    /**
     * Count of DMA descriptors for DAC config
     */
    static constexpr int DAC_CFG_DESC_NUM = 2;

    /**
     * DMA buffer size for DAC, should be 32-4092, multiple of 4
     */
    static constexpr size_t DAC_CFG_BUFFER_SIZE = 1 << 8;

    /**
     * Sample rate in Hz for DAC
     */
    static constexpr int DAC_CFG_SAMPLE_RATE = 14000;

    /**
     * Offset of DAC data, -128 to 127
     */
    static constexpr int DAC_CFG_OFFSET = 0;

    /**
     * Max number of events in the event queue that is populated in ISR,
     * oldest item is dropped once exceeded
     */
    static constexpr size_t DAC_WRITE_QUEUE_CAPACITY = 8;

    /**
     * Max number of events in the event queue that contains changes to tone  data,
     * oldest item is dropped once exceeded
     */
    static constexpr size_t TONES_QUEUE_CAPACITY = 1;

    /**
     * Size of the working buffer
     */
    static constexpr size_t WORKING_BUFFER_SIZE = DAC_CFG_BUFFER_SIZE;

    dac_continuous_handle_t handle;
    TaskHandle_t task;

    uint8_t* buffer;

    SemaphoreHandle_t task_started_sem;
    QueueHandle_t tones_queue_handle;
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
            const auto self = static_cast<signal_generator*>(task_param);

            if (self == nullptr)
                throw std::invalid_argument("Invalid task param.");

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

            //Main loop
            try
            {
                FLOGI("{} Task {} starting main loop.", LOG_KEY, TASK_NAME);

                dac_event_data_t ev;
                tone_period_vec_t tone_data {};
                size_t offset = 0; //The number of bytes that have been written so far

                do
                {
                    try
                    {
                        //Await previous DAC write completion. While doing so, recalculate next buffer as changes occur
                        bool should_update_buffer = true;

                        do
                        {
                            tone_vec_t new_tone_data;

                            if (xQueueReceive(self->tones_queue_handle, &new_tone_data, 0))
                            {
                                should_update_buffer = true;
                                tone_data.clear();

                                for (auto i = 0; i < new_tone_data.size(); i++)
                                {
                                    const auto tone = new_tone_data[i];
                                    tone_data.add_to_end({
                                        .tone = tone,
                                        .period = tone.tone.get_period(DAC_CFG_SAMPLE_RATE)
                                    });
                                }
                            }

                            if (should_update_buffer)
                            {
                                should_update_buffer = false;
                                self->update_buffer(tone_data, offset);
                            }
                        }
                        while (!xQueueReceive(self->dac_write_queue_handle, &ev, 10 / portTICK_PERIOD_MS));

                        FVERBOSE("{} Task {} received DAC event.", LOG_KEY, TASK_NAME);

                        size_t loaded = 0;

                        //Try write to DAC
                        if (const auto write_result = dac_continuous_write_asynchronously(self->handle,
                            static_cast<uint8_t*>(ev.buf), ev.buf_size,
                            self->buffer, WORKING_BUFFER_SIZE, &loaded);
                            write_result != ESP_OK)
                        {
                            FLOGE("{} Didn't successfully write to DAC DMA buffer in task {}. [0x{:02}] {}", LOG_KEY, TASK_NAME, write_result, esp_err_to_name(write_result));
                        }

                        FVERBOSE("{} Task {} wrote {} bytes to DAC.", LOG_KEY, TASK_NAME, loaded);

                        //Update offset
                        offset += loaded;
                    }
                    catch (std::exception& e)
                    {
                        FLOGE("{} An exception occurred during task {} main loop: {}", LOG_KEY, TASK_NAME, e.what());
                    }
                }
                while (true);
            }
            catch (std::exception& e)
            {
                FLOGE("{} An exception occurred during task {}: {}", LOG_KEY, TASK_NAME, e.what());
            }

            //Stop write
            do
            {
                FLOGI("{} Task {} stopping DAC continuous write.", LOG_KEY, TASK_NAME);

                if (const auto stop_write_result = dac_continuous_stop_async_writing(self->handle);
                    stop_write_result != ESP_OK)
                {
                    FLOGE("{} Task {} failed to stop DAC async write. [0x{:04X}] {}", LOG_KEY, TASK_NAME, stop_write_result, esp_err_to_name(stop_write_result));
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    continue;
                }

                break;
            }
            while (true);
        }
        catch (const std::exception& e)
        {
            FLOGE("{} An exception occurred while starting task {}: {}", LOG_KEY, TASK_NAME, e.what());
        }

        do { vTaskDelay(portMAX_DELAY); } while (true);
    }

    void update_buffer(const tone_period_vec_t& tone_data, const size_t offset) const
    {
        try
        {
            FVERBOSE("{} Populate working buffer starting at offset {}", LOG_KEY, offset);

            if (tone_data.empty()) //No data given. Clear buffer and reset offset.
            {
                FVERBOSE("{} Clearing working buffer.", LOG_KEY, offset);
                memset(buffer, 0, WORKING_BUFFER_SIZE);
                return;
            }

            for (size_t i = 0; i < WORKING_BUFFER_SIZE; i++)
            {
                //Add together waves for each tone
                uint32_t sum = 0;

                for (auto t = 0; t < tone_data.size(); t++)
                {
                    if (const auto tone_datum = tone_data[t]; !tone_datum.is_zero_or_invalid())
                    {
                        // const auto volume = tone_datum.tone.volume;

                        // sum += static_cast<uint32_t>(std::clamp(
                        //     tone_datum.tone.volume * (sin(M_TWOPI * (i + offset) / tone_datum.period) + 1) / 2,
                        //     0.,
                        //     static_cast<double>(std::numeric_limits<uint32_t>::max())
                        // ));

                        sum += static_cast<uint32_t>(std::clamp(
                          tone_datum.tone.volume * (sin(M_TWOPI * tone_datum.tone.tone.frequency_hz() * (i + offset) / DAC_CFG_SAMPLE_RATE) + 1) / 2,
                          0.,
                          static_cast<double>(std::numeric_limits<uint32_t>::max())
                        ));

                        // if (const auto period = tone_datum.period; ((i + offset) % period) <= period / 2)
                        // {
                        //     sum += volume;
                        // }
                    }
                }

                buffer[i] = static_cast<uint8_t>(std::clamp(sum, 0UL, 0xFFUL));
            }
        }
        catch (std::exception& e)
        {
            throw std::runtime_error(std::format("An exception occurred while populating data buffer: {}", e.what()));
        }
    }

public:
    signal_generator() : handle(nullptr), task(nullptr)
    {
        task_started_sem = xSemaphoreCreateBinary();
        tones_queue_handle = xQueueCreate(TONES_QUEUE_CAPACITY, sizeof(tone_vec_t));
        dac_write_queue_handle = xQueueCreate(DAC_WRITE_QUEUE_CAPACITY, sizeof(dac_event_data_t));

        buffer = new uint8_t[WORKING_BUFFER_SIZE];
    }

    signal_generator(const signal_generator& other) = delete;
    signal_generator(signal_generator&& other) = delete;
    signal_generator& operator=(const signal_generator& other) = delete;
    signal_generator& operator=(signal_generator&& other) = delete;

    void update_tones(const tone_vec_t* data, const TickType_t timeout = portMAX_DELAY) const
    {
        //Enqueue this event
        xQueueSendToBack(tones_queue_handle, data, timeout);
    }

    signal_generator_start_result start()
    {
        try
        {
            if (task != nullptr)
            {
                FLOGW("{} Task {} is already started (0x{:08X})", LOG_KEY, TASK_NAME, reinterpret_cast<uintptr_t>(task));
                return signal_generator_start_result::ALREADY_STARTED;
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
                return signal_generator_start_result::ERR;
            }

            FLOGI("{} Task {} is started (0x{:08X}).", LOG_KEY, TASK_NAME, reinterpret_cast<uintptr_t>(task));

            return signal_generator_start_result::OK;
        }
        catch (const std::exception& e)
        {
            FLOGE("{} An exception occurred while starting task: {}", LOG_KEY, e.what());
        }

        return signal_generator_start_result::ERR;
    }

    [[nodiscard]] static std::unique_ptr<signal_generator> try_create(const dac_channel_mask_t channels = DAC_CHANNEL_MASK_ALL, const dac_continuous_channel_mode_t channel_mode = DAC_CHANNEL_MODE_SIMUL)
    {
        FLOGI("{} Creating signal generator.", LOG_KEY);

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

        auto gtor = std::make_unique<signal_generator>();
        gtor->handle = handle;

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

        if (const auto register_cb_result = dac_continuous_register_event_callback(handle, &cb, gtor->dac_write_queue_handle);
            register_cb_result != ESP_OK)
        {
            FLOGE("{} Failed to register DAC callbacks. [0x{:04X}] {}", LOG_KEY, register_cb_result, esp_err_to_name(register_cb_result));
            return nullptr;
        }

        return gtor;
    }

    ~signal_generator()
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

#endif //AUDIO_CONTROLLER_SIGNAL_GENERATOR_H
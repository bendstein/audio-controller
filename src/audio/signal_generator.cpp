//
// Created by bendstein on 11/2/2026.
//
#include "app_common.h"
#include "audio/signal_generator.h"

signal_generator::signal_generator(const signal_generator_param& param)
    : dac_handle(nullptr), dac_handle1(nullptr), timer_handle(nullptr), task(nullptr), sample_rate_hz(param.sample_rate_hz)
{
    timer_queue_handle = xQueueCreate(param.timer_queue_size, sizeof(gptimer_alarm_event_data_t));
    tones_queue_handle = xQueueCreate(1, sizeof(signal_generator_tone*));
}

signal_generator::~signal_generator()
{
    //Dealloc DAC/task handles on destroy
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

    if (dac_handle != nullptr)
    {
        try
        {
            FLOGI("{} Removing DAC channel 0 handle 0x{:08X}.",
                 LOG_KEY,
                 reinterpret_cast<uintptr_t>(dac_handle));

            if (const auto remove_dac_channel_result = dac_oneshot_del_channel(dac_handle);
                remove_dac_channel_result != ESP_OK)
            {
                FLOGE("{} Failed to remove DAC channel 0 handle 0x{:08X}. [0x{:04X}] {}",
                    LOG_KEY,
                    reinterpret_cast<uintptr_t>(dac_handle),
                    remove_dac_channel_result,
                    esp_err_to_name(remove_dac_channel_result));
            }
        }
        catch (std::exception& e)
        {
            FLOGE("{} An exception occurred while removing DAC channel 0 handle 0x{:08X}: {}",
                LOG_KEY, reinterpret_cast<uintptr_t>(dac_handle), e.what());
        }
    }

    if (dac_handle1 != nullptr)
    {
        try
        {
            FLOGI("{} Removing DAC channel 1 handle 0x{:08X}.",
                 LOG_KEY,
                 reinterpret_cast<uintptr_t>(dac_handle1));

            if (const auto remove_dac_channel_result = dac_oneshot_del_channel(dac_handle1);
                remove_dac_channel_result != ESP_OK)
            {
                FLOGE("{} Failed to remove DAC channel 1 handle 0x{:08X}. [0x{:04X}] {}",
                    LOG_KEY,
                    reinterpret_cast<uintptr_t>(dac_handle1),
                    remove_dac_channel_result,
                    esp_err_to_name(remove_dac_channel_result));
            }
        }
        catch (std::exception& e)
        {
            FLOGE("{} An exception occurred while removing DAC channel 1 handle 0x{:08X}: {}",
                LOG_KEY, reinterpret_cast<uintptr_t>(dac_handle1), e.what());
        }
    }
}

bool signal_generator::timer_isr(gptimer_handle_t, const gptimer_alarm_event_data_t* ev, void* isr_param)
{
    //ISR adds event to the queue, does nothing else
    BaseType_t should_wake = pdFALSE;

    if (isr_param == nullptr)
        return should_wake;

    const auto queue = static_cast<QueueHandle_t>(isr_param);

    if (xQueueIsQueueFullFromISR(queue)) //Dequeue oldest item on full
    {
        gptimer_alarm_event_data_t dummy;
        xQueueReceiveFromISR(queue, &dummy, &should_wake);
    }

    //Enqueue this event
    xQueueSendFromISR(queue, ev, &should_wake);

    return should_wake;
}

void signal_generator::dac_task(void* task_param)
{
    assert(task_param != nullptr);

    try
    {
        const auto self = static_cast<signal_generator*>(task_param);

        if (self == nullptr)
            throw std::invalid_argument("Invalid task param.");

        const auto increment_by = TIMER_RESOLUTION_HZ / self->sample_rate_hz;

        gptimer_alarm_event_data_t ev;

        bool prev_states[TONE_DATA_CAPACITY] {};
        uint64_t ticks[TONE_DATA_CAPACITY] {};
        uint32_t half_periods[TONE_DATA_CAPACITY] {};
        signal_generator_tone tones[TONE_DATA_CAPACITY] {};
        signal_generator_tone next_tones[TONE_DATA_CAPACITY] {};

        FLOGI("{} Task {} starting main loop.", LOG_KEY, TASK_NAME);

        while (true)
        {
            //Get updated tone data while waiting for tick. Calculate next value on initial iteration, and as tone data is updated
            uint8_t next = 0;
            bool recalculate_next = true;

            do
            {
                signal_generator_tone* new_tones;

                if (const auto result = xQueueReceive(self->tones_queue_handle, &new_tones, 0);
                    result == pdTRUE)
                {
                    VERBOSE("Received new tone data.");
                    recalculate_next = true; //Tone data changed, so next should be recalculated

                    //Set new tone data into next tones
                    for (auto i = 0; i < TONE_DATA_CAPACITY; i++)
                    {
                        next_tones[i] = new_tones[i];

                        //If corresponding tick is 0, reset state and update tone immediately
                        if (ticks[i] == 0 || tones[i].frequency_hz == 0 || tones[i].volume == 0)
                        {
                            tones[i] = next_tones[i];
                            half_periods[i] = musical_note_tone::get_period(tones[i].frequency_hz * 2, TIMER_RESOLUTION_HZ);
                            prev_states[i] = false;
                        }
                    }
                }

                if (recalculate_next)
                {
                    recalculate_next = false;
                    next = 0;

                    for (auto i = 0; i < TONE_DATA_CAPACITY; i++)
                    {
                        const auto [frequency_hz, volume] = tones[i];

                        if (frequency_hz == 0 || volume == 0) //Skip zero
                            continue;

                        const auto half_period = half_periods[i];
                        const auto prev_state = prev_states[i];
                        const auto tick = ticks[i];
                        const auto next_state = (tick / half_period) % 2 == 1;

                        //Handle state change
                        if (prev_state != next_state)
                        {
                            prev_states[i] = next_state;

                            //If state change from 1 to 0, cycle complete. Reset state and allow tone change
                            if (!next_state)
                            {
                                ticks[i] = 0;
                                tones[i] = next_tones[i];
                                half_periods[i] = musical_note_tone::get_period(tones[i].frequency_hz * 2, TIMER_RESOLUTION_HZ);
                            }
                        }

                        //If state is 1, add volume
                        if (next_state)
                            next += volume;
                    }
                }
            } while (!xQueueReceive(self->timer_queue_handle, &ev, 1));

            //Write next value
            if (const auto result = dac_oneshot_output_voltage(self->dac_handle, next);
                result != ESP_OK)
            {
                FLOGE("{} Task {} didn't successfully write value {} to DAC channel 0. [0x{:02}] {}" ,
                    LOG_KEY, TASK_NAME, next, result, esp_err_to_name(result));
            }

            if (const auto result = dac_oneshot_output_voltage(self->dac_handle1, next);
                result != ESP_OK)
            {
                FLOGE("{} Task {} didn't successfully write value {} to DAC channel 1. [0x{:02}] {}" ,
                    LOG_KEY, TASK_NAME, next, result, esp_err_to_name(result));
            }

            //Increment ticks
            for (auto i = 0; i < TONE_DATA_CAPACITY; i++)
            {
                const auto [frequency_hz, volume] = tones[i];

                //Reset ticks if 0
                if (frequency_hz == 0 || volume == 0)
                    ticks[i] = 0;
                else
                    ticks[i] += increment_by;
            }
        }
    }
    catch (const std::exception& e)
    {
        FLOGE("{} An exception occurred while starting task {}: {}" , LOG_KEY, TASK_NAME, e.what());
    }

    do { vTaskDelay(portMAX_DELAY); } while (true);
}

esp_err_t signal_generator::start()
{
    try
    {
        if (task != nullptr)
        {
            FLOGW("{} Task {} is already started (0x{:08X})", LOG_KEY, TASK_NAME, reinterpret_cast<uintptr_t>(task));
        }
        else
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
                FLOGE("{} Failed to start task {}: 0x{:04X}", LOG_KEY, TASK_NAME, static_cast<uint32_t>(result));
                return ESP_FAIL;
            }

            FLOGI("{} Task {} is started (0x{:08X}).", LOG_KEY, TASK_NAME, reinterpret_cast<uintptr_t>(task));
        }

        if (timer_handle == nullptr)
        {
            FLOGE("{} Cannot start nullptr timer.", LOG_KEY);
            return ESP_ERR_INVALID_STATE;
        }

        if (const auto result = gptimer_start(timer_handle);
            result != ESP_OK)
        {
            FLOGE("{} Failed to start timer: 0x{:04X}", LOG_KEY, static_cast<uint32_t>(result));
            return result;
        }

        FLOGI("{} Timer is started.", LOG_KEY);

        return ESP_OK;
    }
    catch (const std::exception& e)
    {
        FLOGE("{} An exception occurred while starting task: {}" , LOG_KEY, e.what());
    }

    return ESP_FAIL;
}

bool signal_generator::update_tones(const signal_generator_tone data[TONE_DATA_CAPACITY], const TickType_t timeout)
{
    VERBOSE("Updating tone data.");

    if (const auto result = xQueueSendToBack(tones_queue_handle, &data, timeout) == pdTRUE;
        result)
    {
        VERBOSE("Sent updated tone data.");
        return result;
    }
    else
    {
        return result;
    }
}

std::unique_ptr<signal_generator> signal_generator::try_create(const signal_generator_param& param)
{
    FLOGI("{} Creating signal generator.", LOG_KEY);

    auto gtor = std::unique_ptr<signal_generator>(new signal_generator(param));

    constexpr dac_oneshot_config_t dac_config = {
        .chan_id = DAC_CHAN_0
    };

    constexpr dac_oneshot_config_t dac_config1 = {
        .chan_id = DAC_CHAN_1
    };

    if (const auto result = dac_oneshot_new_channel(&dac_config, &gtor->dac_handle);
        result != ESP_OK)
    {
        FLOGE("{} Failed to allocate DAC channel 0. [0x{:04X}] {}" , LOG_KEY, result, esp_err_to_name(result));
        return nullptr;
    }

    if (const auto result = dac_oneshot_new_channel(&dac_config1, &gtor->dac_handle1);
        result != ESP_OK)
    {
        FLOGE("{} Failed to allocate DAC channel 1. [0x{:04X}] {}" , LOG_KEY, result, esp_err_to_name(result));
        return nullptr;
    }

    constexpr gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
        .intr_priority = 0,
        .flags = {
            .intr_shared = 0,
            .allow_pd = 0
        }
    };

    if (const auto result = gptimer_new_timer(&timer_config, &gtor->timer_handle);
        result != ESP_OK)
    {
        FLOGE("{} Failed to allocate timer. [0x{:04X}] {}" , LOG_KEY, result, esp_err_to_name(result));
        return nullptr;
    }

    assert(param.sample_rate_hz > 0 && param.sample_rate_hz <= TIMER_RESOLUTION_HZ);

    const gptimer_alarm_config_t alarm_config = {
        .alarm_count = TIMER_RESOLUTION_HZ / param.sample_rate_hz,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true
        }
    };

    constexpr gptimer_event_callbacks_t callbacks = {
        .on_alarm = timer_isr
    };

    if (const auto result = gptimer_register_event_callbacks(gtor->timer_handle, &callbacks, gtor->timer_queue_handle);
        result != ESP_OK)
    {
        FLOGE("{} Failed to register timer callbacks. [0x{:04X}] {}" , LOG_KEY, result, esp_err_to_name(result));
        return nullptr;
    }

    if (const auto result = gptimer_set_alarm_action(gtor->timer_handle, &alarm_config);
        result != ESP_OK)
    {
        FLOGE("{} Failed to configure timer alarm. [0x{:04X}] {}" , LOG_KEY, result, esp_err_to_name(result));
        return nullptr;
    }

    if (const auto result = gptimer_enable(gtor->timer_handle);
        result != ESP_OK)
    {
        FLOGE("{} Failed to enable timer. [0x{:04X}] {}" , LOG_KEY, result, esp_err_to_name(result));
        return nullptr;
    }

    return gtor;
}
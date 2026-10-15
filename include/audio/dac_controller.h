//
// Created by bendstein on 1/13/2026.
//

#ifndef AUDIO_CONTROLLER_DAC_CONTROLLER_H
#define AUDIO_CONTROLLER_DAC_CONTROLLER_H
#include <memory>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/dac_continuous.h>

#include "fixed_vec.h"
#include "tones.h"

enum struct dac_controller_start_result
{
    OK,
    ERR,
    ALREADY_STARTED
};

struct dac_controller_tone
{
    musical_note_tone_volume tone;
    uint32_t period;

    [[nodiscard]] bool is_zero() const { return period == 0 || tone.is_zero(); }
    [[nodiscard]] bool is_invalid() const { return tone.is_invalid(); }
    [[nodiscard]] bool is_zero_or_invalid() const { return period == 0 || tone.is_invalid(); }

    bool operator==(const dac_controller_tone& other) const { return tone == other.tone && period == other.period; }

    [[nodiscard]] static consteval dac_controller_tone create_zero() { return dac_controller_tone(musical_note_tone_volume::create_zero(), 0); }
    [[nodiscard]] static consteval dac_controller_tone create_invalid() { return dac_controller_tone(musical_note_tone_volume::create_invalid(), 0); }
};

class dac_controller
{
public:
    static constexpr auto LOG_KEY = "[DAC Ctrl]";
    static constexpr size_t TONE_DATA_CAPACITY = 32;
private:
    static constexpr auto TASK_NAME = "<DAC Ctrl>";
    static constexpr auto TASK_PRIORITY = 8;
    static constexpr auto TASK_STACK_SIZE = 0x1000;
    static constexpr size_t TASK_MESSAGE_QUEUE_INDEX_TONE_CHANGE = 0;

    /**
     * Count of DMA descriptors for DAC config,
     * should be at least 5
     */
    static constexpr int DAC_CFG_DESC_NUM = 1 << 2;

    /**
     * DMA buffer size for DAC, should be 32-4092, multiple of 4
     */
    static constexpr size_t DAC_CFG_BUFFER_SIZE = 1 << 6;

    /**
     * Sample rate in Hz for DAC
     */
    static constexpr int DAC_CFG_SAMPLE_RATE = 1 << 16;

    /**
     * Offset of DAC data, -128 to 127
     */
    static constexpr int DAC_CFG_OFFSET = 0;

    /**
     * Moving average as filter when populating DAC buffer
     */
    static constexpr size_t MOVING_AVERAGE_LEN = 0;

    /**
     * Buffer where the values for the given tones
     * are summed, later transferred to DAC buffer
     */
    static constexpr size_t BUFFER_LEN = 4 * DAC_CFG_BUFFER_SIZE;

    /**
     * 1 / duty cycle for square wave
     */
    static constexpr uint8_t DUTY_CYCLE = 2;

    /**
     * Period is multiplied by this to maintain some precision
     * while still storing as an int
     */
    static constexpr uint32_t PERIOD_FACTOR = 10;

    /**
     * Max number of events in the event queue that is populated in ISR,
     * oldest item is dropped once exceeded
     */
    static constexpr size_t DAC_WRITE_QUEUE_CAPACITY = 4;

    dac_continuous_handle_t handle;
    TaskHandle_t task;

    uint8_t* buffer;

    fixed_vec<musical_note_tone_volume, TONE_DATA_CAPACITY> tone_data {};

    SemaphoreHandle_t tones_mux;
    SemaphoreHandle_t task_started_sem;
    QueueHandle_t dac_write_queue_handle;

    static bool IRAM_ATTR dac_isr(dac_continuous_handle_t, const dac_event_data_t* ev, void* isr_param);
    [[noreturn]] static void dac_task(void* task_param);

    /**
     * @param buf data buffer to write into. Is mutable.
     * @param buf_len Length of data buffer. Not mutated.
     * @param offset Current offset int buf. Is mutated with new offset after write.
     * @param tones Tones to write. Not mutated.
     */
    void populate_buffer(uint8_t* buf, size_t buf_len, size_t& offset, const fixed_vec<dac_controller_tone, TONE_DATA_CAPACITY>& tones) const;
public:
    dac_controller();

    dac_controller(const dac_controller& other) = delete;
    dac_controller(dac_controller&& other) = delete;
    dac_controller& operator=(const dac_controller& other) = delete;
    dac_controller& operator=(dac_controller&& other) = delete;

    dac_controller_start_result start();
    void write(const fixed_vec<musical_note_tone_volume, TONE_DATA_CAPACITY>& data);
    [[nodiscard]] static std::unique_ptr<dac_controller> try_create(dac_channel_mask_t channels = DAC_CHANNEL_MASK_ALL, dac_continuous_channel_mode_t channel_mode = DAC_CHANNEL_MODE_SIMUL);

    ~dac_controller();
};

#endif //AUDIO_CONTROLLER_DAC_CONTROLLER_H
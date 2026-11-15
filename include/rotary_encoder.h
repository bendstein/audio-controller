//
// Created by bendstein on 11/15/2026.
//

#ifndef AUDIO_CONTROLLER_ROTARY_ENCODER_H
#define AUDIO_CONTROLLER_ROTARY_ENCODER_H
#include <memory>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <soc/gpio_num.h>

struct rotary_encoder_param
{
    gpio_num_t pin_button;
    gpio_num_t pin_encoder_l;
    gpio_num_t pin_encoder_r;
    size_t isr_queue_len;
};

enum struct rotary_encoder_isr_queue_item : uint8_t
{
    NONE,
    BUTTON_FALLING,
    BUTTON_RISING,
    ENCODER_L_FALLING,
    ENCODER_L_RISING,
    ENCODER_R_FALLING,
    ENCODER_R_RISING
};

class rotary_encoder
{
public:
    static constexpr auto LOG_KEY = "[Rotary Enc.]";
private:
    static constexpr auto TASK_NAME = "<ROTARY-ENC>";
    static constexpr auto TASK_PRIORITY = 1;
    static constexpr auto TASK_STACK_SIZE = 0x1000;

    gpio_num_t pin_button;
    gpio_num_t pin_encoder_l;
    gpio_num_t pin_encoder_r;
    TaskHandle_t task_handle;
    QueueHandle_t isr_queue;

    static bool is_pin_valid(const gpio_num_t pin) { return pin != GPIO_NUM_NC && pin < GPIO_NUM_MAX; }

    void IRAM_ATTR isr_handler(const rotary_encoder_isr_queue_item* ev) const;
    static void IRAM_ATTR isr_handler_pin_button(void* isr_param);
    static void IRAM_ATTR isr_handler_pin_encoder_l(void* isr_param);
    static void IRAM_ATTR isr_handler_pin_encoder_r(void* isr_param);

    [[noreturn]] static void task(void* task_param);

    explicit rotary_encoder(const rotary_encoder_param& param);
public:
    rotary_encoder() = delete;
    rotary_encoder(const rotary_encoder& other) = delete;
    rotary_encoder(rotary_encoder&& other) = delete;
    rotary_encoder& operator=(const rotary_encoder& other) = delete;
    rotary_encoder& operator=(rotary_encoder&& other) = delete;

    [[nodiscard]] bool is_button_enabled() const { return is_pin_valid(pin_button); }
    [[nodiscard]] bool is_rotate_enabled() const { return is_pin_valid(pin_encoder_l) && is_pin_valid(pin_encoder_r); }

    [[nodiscard]] static std::unique_ptr<rotary_encoder> try_create(const rotary_encoder_param& param);

    ~rotary_encoder();
};

#endif //AUDIO_CONTROLLER_ROTARY_ENCODER_H
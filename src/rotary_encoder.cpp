//
// Created by bendstein on 11/15/2026.
//
#include "app_common.h"
#include "rotary_encoder.h"

rotary_encoder_state_machine_result rotary_encoder_state_machine::transition(const rotary_encoder_state_machine_token token)
{
    switch (token)
    {
        case rotary_encoder_state_machine_token::BUTTON_FALLING:
            //Button was pressed down
            state = rotary_encoder_state_machine_state::BUTTON_DOWN;
            last_result = rotary_encoder_state_machine_result::BUTTON_DOWN;
            break;
        case rotary_encoder_state_machine_token::BUTTON_RISING:
            //Button was released
            switch (state)
            {
                case rotary_encoder_state_machine_state::BUTTON_DOWN:
                    //Button was pressed and released
                    state = rotary_encoder_state_machine_state::START;
                    last_result = rotary_encoder_state_machine_result::BUTTON_UP;
                    break;
                default:
                    //Invalid transition
                    state = rotary_encoder_state_machine_state::START;
                    last_result = rotary_encoder_state_machine_result::NONE;
                    break;
            }
            break;
        case rotary_encoder_state_machine_token::ENCODER_L_FALLING:
            //Encoder left pin has started contact
            switch (state)
            {
                case rotary_encoder_state_machine_state::R_FALLING:
                    //R falling, L falling; Potential CCW turn
                    state = rotary_encoder_state_machine_state::R_FALLING_L_FALLING;
                    last_result = rotary_encoder_state_machine_result::NONE;
                    break;
                case rotary_encoder_state_machine_state::START:
                default:
                    //Otherwise, move to l falling state
                    state = rotary_encoder_state_machine_state::L_FALLING;
                    last_result = rotary_encoder_state_machine_result::NONE;
                    break;
            }
            break;
        case rotary_encoder_state_machine_token::ENCODER_L_RISING:
            //Encoder left pin has stopped contact
            switch (state)
            {
                case rotary_encoder_state_machine_state::L_FALLING_R_FALLING:
                    //L falling, R falling, L rising; Potential CW turn
                    state = rotary_encoder_state_machine_state::L_FALLING_R_FALLING_L_RISING;
                    last_result = rotary_encoder_state_machine_result::NONE;
                    break;
                case rotary_encoder_state_machine_state::R_FALLING_L_FALLING_R_RISING:
                    //R falling, L falling, R rising, L rising; Full CCW turn
                    state = rotary_encoder_state_machine_state::START;
                    last_result = rotary_encoder_state_machine_result::ENCODER_CCW;
                    break;
                default:
                    //Invalid transition
                    state = rotary_encoder_state_machine_state::START;
                    last_result = rotary_encoder_state_machine_result::NONE;
                    break;
            }
            break;
        case rotary_encoder_state_machine_token::ENCODER_R_FALLING:
            //Encoder right pin has started contact
            switch (state)
            {
                case rotary_encoder_state_machine_state::L_FALLING:
                    //L falling, R falling; Potential CW turn
                    state = rotary_encoder_state_machine_state::L_FALLING_R_FALLING;
                    last_result = rotary_encoder_state_machine_result::NONE;
                    break;
                case rotary_encoder_state_machine_state::START:
                default:
                    //Otherwise, move to r falling state
                    state = rotary_encoder_state_machine_state::R_FALLING;
                    last_result = rotary_encoder_state_machine_result::NONE;
                    break;
            }
            break;
        case rotary_encoder_state_machine_token::ENCODER_R_RISING:
            //Encoder left pin has stopped contact
            switch (state)
            {
                case rotary_encoder_state_machine_state::R_FALLING_L_FALLING:
                    //R falling, L falling, R rising; Potential CCW turn
                    state = rotary_encoder_state_machine_state::R_FALLING_L_FALLING_R_RISING;
                    last_result = rotary_encoder_state_machine_result::NONE;
                    break;
                case rotary_encoder_state_machine_state::L_FALLING_R_FALLING_L_RISING:
                    //L falling, R falling, L rising, R rising; Full CW turn
                    state = rotary_encoder_state_machine_state::START;
                    last_result = rotary_encoder_state_machine_result::ENCODER_CW;
                    break;
                default:
                    //Invalid transition
                    state = rotary_encoder_state_machine_state::START;
                    last_result = rotary_encoder_state_machine_result::NONE;
                    break;
            }
            break;
        case rotary_encoder_state_machine_token::NONE:
        default:
            //Invalid transition
            state = rotary_encoder_state_machine_state::START;
            last_result = rotary_encoder_state_machine_result::NONE;
            break;
    }

    return get_last_result();
}

rotary_encoder::rotary_encoder(const rotary_encoder_param& param)
{
    state_machine = {};
    pin_button = param.pin_button;
    pin_encoder_l = param.pin_encoder_l;
    pin_encoder_r = param.pin_encoder_r;
    task_handle = nullptr;
    isr_queue = xQueueCreate(param.isr_queue_len, sizeof(rotary_encoder_state_machine_token));
    event_queue = xQueueCreate(param.event_queue_len, sizeof(rotary_encoder_state_machine_result));
    event_queue_timeout = param.event_queue_timeout;
}

rotary_encoder::~rotary_encoder()
{
    //Dealloc queue, stop tasks, isrs on destroy
    if (isr_queue != nullptr)
    {
        vQueueDelete(isr_queue);
    }

    if (task_handle != nullptr)
    {
        vTaskDelete(task_handle);
    }

    if (is_button_enabled())
    {
        if (const auto result = gpio_isr_handler_remove(pin_button);
            result != ESP_OK)
        {
            FLOGE("{} Failed to remove isr from button pin ({}). (0x{:04X})", LOG_KEY, static_cast<int8_t>(pin_button), result);
        }
    }

    if (is_rotate_enabled())
    {
        if (const auto result = gpio_isr_handler_remove(pin_encoder_l);
            result != ESP_OK)
        {
            FLOGE("{} Failed to remove isr from left encoder pin ({}). (0x{:04X})", LOG_KEY, static_cast<int8_t>(pin_encoder_l), result);
        }

        if (const auto result = gpio_isr_handler_remove(pin_encoder_r);
            result != ESP_OK)
        {
            FLOGE("{} Failed to remove isr from right encoder pin ({}). (0x{:04X})", LOG_KEY, static_cast<int8_t>(pin_encoder_r), result);
        }
    }
}

void rotary_encoder::isr_handler(const rotary_encoder_state_machine_token* ev) const
{
    if (isr_queue == nullptr)
        return;

    if (xQueueIsQueueFullFromISR(isr_queue)) //Ignore if full
        return;

    //Enqueue this event
    xQueueSendFromISR(isr_queue, ev, nullptr);
}

void rotary_encoder::isr_handler_pin_button(void* isr_param)
{
    const auto self = static_cast<rotary_encoder*>(isr_param);

    if (self == nullptr)
        return;

    const auto message = DIGITAL_BOOL(gpio_get_level(self->pin_button))
        ? rotary_encoder_state_machine_token::BUTTON_RISING
        : rotary_encoder_state_machine_token::BUTTON_FALLING;

    self->isr_handler(&message);
}

void rotary_encoder::isr_handler_pin_encoder_l(void* isr_param)
{
    const auto self = static_cast<rotary_encoder*>(isr_param);

    if (self == nullptr)
        return;

    const auto message = DIGITAL_BOOL(gpio_get_level(self->pin_encoder_l))
        ? rotary_encoder_state_machine_token::ENCODER_L_RISING
        : rotary_encoder_state_machine_token::ENCODER_L_FALLING;

    self->isr_handler(&message);
}

void rotary_encoder::isr_handler_pin_encoder_r(void* isr_param)
{
    const auto self = static_cast<rotary_encoder*>(isr_param);

    if (self == nullptr)
        return;

    const auto message = DIGITAL_BOOL(gpio_get_level(self->pin_encoder_r))
        ? rotary_encoder_state_machine_token::ENCODER_R_RISING
        : rotary_encoder_state_machine_token::ENCODER_R_FALLING;

    self->isr_handler(&message);
}

void rotary_encoder::task(void* task_param)
{
    assert(task_param != nullptr);

    try
    {
        const auto self = static_cast<rotary_encoder*>(task_param);

        if (self == nullptr)
            throw std::invalid_argument("Invalid task param.");

        while (true)
        {
            try
            {
                rotary_encoder_state_machine_token ev;
                if (const auto result = xQueueReceive(self->isr_queue, &ev, portMAX_DELAY);
                    result != pdTRUE)
                {
                    FLOGW("{} Failed to read from isr queue during task {}: 0x{:04X}", LOG_KEY, TASK_NAME, result);
                    continue;
                }

                //Advance state machine
                const auto result = self->state_machine.transition(ev);

                switch (result)
                {
                    case rotary_encoder_state_machine_result::BUTTON_DOWN:
                        FLOGI("BUTTON_DOWN");
                        break;
                    case rotary_encoder_state_machine_result::BUTTON_UP:
                        FLOGI("BUTTON_UP");
                        break;
                    case rotary_encoder_state_machine_result::ENCODER_CW:
                        FLOGI("ENCODER_CW");
                        break;
                    case rotary_encoder_state_machine_result::ENCODER_CCW:
                        FLOGI("ENCODER_CCW");
                        break;
                    default:
                        break;
                }

                //Enqueue event if space
                if (const auto enqueue_result = xQueueSendToBack(self->event_queue, &result, self->event_queue_timeout);
                    enqueue_result != pdTRUE)
                {
                    FLOGW("{} Failed to enqueue rotary encoder event to event queue during task {}. (0x{:04X})", LOG_KEY, TASK_NAME, enqueue_result);
                }
            }
            catch (const std::exception& e)
            {
                FLOGE("{} An exception occurred during task {} main loop: {}", LOG_KEY, TASK_NAME, e.what());
            }
        }
    }
    catch (const std::exception& e)
    {
        FLOGE("{} An exception occurred while starting task {}: {}" , LOG_KEY, TASK_NAME, e.what());
    }

    do { vTaskDelay(portMAX_DELAY); } while (true);
}

std::unique_ptr<rotary_encoder> rotary_encoder::try_create(const rotary_encoder_param& param)
{
    FLOGI("{} Creating rotary encoder (B: {}, L: {}, R: {}).", LOG_KEY,
        static_cast<int8_t>(param.pin_button), static_cast<int8_t>(param.pin_encoder_l), static_cast<int8_t>(param.pin_encoder_r));

    auto rotary = std::make_unique<rotary_encoder>(param);

    if (rotary->isr_queue == nullptr)
    {
        FLOGE("{} Failed to create ISR queue.", LOG_KEY);
        return nullptr;
    }

    if (rotary->event_queue == nullptr)
    {
        FLOGE("{} Failed to create event queue.", LOG_KEY);
        return nullptr;
    }

    if (rotary->is_button_enabled())
    {
        const gpio_config_t pin_button_config = {
            .pin_bit_mask = 1ULL << static_cast<int8_t>(rotary->pin_button),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE
        };

        if (const auto result = gpio_config(&pin_button_config);
            result != ESP_OK)
        {
            FLOGE("{} Failed to configure button pin ({}). (0x{:04X})", LOG_KEY, static_cast<int8_t>(rotary->pin_button), result);
            return nullptr;
        }

        if (const auto result = gpio_isr_handler_add(rotary->pin_button, isr_handler_pin_button, rotary.get());
            result != ESP_OK)
        {
            FLOGE("{} Failed to attach isr to button pin ({}). (0x{:04X})", LOG_KEY, static_cast<int8_t>(rotary->pin_button), result);
            return nullptr;
        }
    }
    else
    {
        FLOGW("{} Will not configure button pin ({}).", LOG_KEY, static_cast<int8_t>(rotary->pin_button));
    }

    if (rotary->is_rotate_enabled())
    {
        const gpio_config_t pin_encoder_lr_config = {
            .pin_bit_mask = (1ULL << static_cast<int8_t>(rotary->pin_encoder_l)) | (1ULL << static_cast<int8_t>(rotary->pin_encoder_r)),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE
        };

        if (const auto result = gpio_config(&pin_encoder_lr_config);
            result != ESP_OK)
        {
            FLOGE("{} Failed to configure encoder pins (l {}/r {}). (0x{:04X})", LOG_KEY,
                static_cast<int8_t>(rotary->pin_encoder_l), static_cast<int8_t>(rotary->pin_encoder_r), result);
            return nullptr;
        }

        if (const auto result = gpio_isr_handler_add(rotary->pin_encoder_l, isr_handler_pin_encoder_l, rotary.get());
            result != ESP_OK)
        {
            FLOGE("{} Failed to attach isr to left encoder pin ({}). (0x{:04X})", LOG_KEY, static_cast<int8_t>(rotary->pin_encoder_l), result);
            return nullptr;
        }

        if (const auto result = gpio_isr_handler_add(rotary->pin_encoder_r, isr_handler_pin_encoder_r, rotary.get());
            result != ESP_OK)
        {
            FLOGE("{} Failed to attach isr to right encoder pin ({}). (0x{:04X})", LOG_KEY, static_cast<int8_t>(rotary->pin_encoder_r), result);
            return nullptr;
        }
    }
    else
    {
        FLOGW("{} Will not configure encoder pins (l {}/r {}).", LOG_KEY, static_cast<int8_t>(rotary->pin_encoder_l), static_cast<int8_t>(rotary->pin_encoder_r));
    }

    if (const auto result = xTaskCreate(
        task,
        TASK_NAME,
        TASK_STACK_SIZE,
        rotary.get(),
        TASK_PRIORITY,
        &rotary->task_handle
    ); result != pdPASS)
    {
        FLOGE("{} Failed to start task {}: 0x{:04X}", LOG_KEY, TASK_NAME, static_cast<uint32_t>(result));
        return nullptr;
    }

    FLOGI("{} Task {} is started (0x{:08X}).", LOG_KEY, TASK_NAME, reinterpret_cast<uintptr_t>(rotary->task_handle));

    FLOGI("{} Successfully created rotary encoder.", LOG_KEY);

    return rotary;
}
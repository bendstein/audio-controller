//
// Created by bendstein on 12/25/2025.
//

#ifndef AUDIO_CONTROLLER_ASYNC_STATE_MACHINE_H
#define AUDIO_CONTROLLER_ASYNC_STATE_MACHINE_H
#include <atomic>
#include <cstdint>
#include <esp_err.h>
#include <format>
#include <mutex>
#include <driver/i2c_master.h>

#include "i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

enum struct i2c_async_state_machine_state : uint8_t
{
    none,
    starting,
    started,
    completed,
    faulted,
    cancelled,
    invalid
};

enum struct i2c_async_state_machine_token_type : uint8_t
{
    none,
    start,
    status_code,
    cancel
};

enum struct i2c_async_state_machine_op : uint8_t
{
    read,
    write
};

struct i2c_async_state_machine_token
{
    i2c_async_state_machine_token_type token_type = i2c_async_state_machine_token_type::none;
    int maybe_value = 0;
};

class i2c_async_state_machine
{
    static std::atomic_int32_t N;

    i2c_async_state_machine_state state = i2c_async_state_machine_state::none;
    uint8_t exit_code = ESP_OK;
    i2c_async_state_machine_op type = i2c_async_state_machine_op::read;
    uint8_t* rw_buffer = nullptr;
    size_t rw_buffer_size = 0;
    i2c_master_dev_handle_t device_handle;
    i2c_master_event_callbacks_t callbacks = {};
    TaskHandle_t task_handle = nullptr;
    std::optional<std::function<i2c_async_state_machine*(i2c_async_state_machine* self_ptr)>> continuation = std::nullopt;
    std::mutex transition_mutex = {};

    void transition(const i2c_async_state_machine_token token)
    {
        const auto lock = std::scoped_lock(transition_mutex);
        switch (state)
        {
            case i2c_async_state_machine_state::none: //Not yet started
                switch (token.token_type)
                {
                    case i2c_async_state_machine_token_type::start: //(none) --start-> (starting)
                        state = i2c_async_state_machine_state::starting;
                        on_start_internal(token);
                    break;
                    case i2c_async_state_machine_token_type::cancel: //(none) --cancel-> (cancelled)
                        state = i2c_async_state_machine_state::cancelled;
                        on_cancel_internal(token);
                    break;
                    default: //Otherwise, token has no effect
                    break;
                }
            break;
            case i2c_async_state_machine_state::starting: //In process of starting
                switch (token.token_type)
                {
                    case i2c_async_state_machine_token_type::status_code:
                        switch (token.maybe_value)
                        {
                            case ESP_OK: //(starting) --status:ok-> (started)
                                state = i2c_async_state_machine_state::started;
                            break;
                            case ESP_ERR_TIMEOUT: //(starting) --status:timeout-> (cancelled)
                                state = i2c_async_state_machine_state::cancelled;
                                on_cancel_internal(token);
                            break;
                            default: //(starting) --status:*-> (faulted)
                                state = i2c_async_state_machine_state::faulted;
                                on_fault_internal(token);
                            break;
                        }
                    break;
                    case i2c_async_state_machine_token_type::cancel: //(starting) --cancel-> (cancelled)
                        state = i2c_async_state_machine_state::cancelled;
                        on_cancel_internal(token);
                    break;
                    default: //Otherwise, token has no effect
                    break;
                }
            break;
            case i2c_async_state_machine_state::started: //Started/running, but not complete
                switch (token.token_type)
                {
                    case i2c_async_state_machine_token_type::status_code:
                        switch (token.maybe_value)
                        {
                            case ESP_OK: //(started) --status:ok-> (complete)
                                state = i2c_async_state_machine_state::completed;
                                break;
                            case ESP_ERR_TIMEOUT: //(started) --status:timeout-> (cancelled)
                                state = i2c_async_state_machine_state::cancelled;
                                on_cancel_internal(token);
                                break;
                            default: //(started) --status:*-> (faulted)
                                state = i2c_async_state_machine_state::faulted;
                                on_fault_internal(token);
                                break;
                        }
                        break;
                    case i2c_async_state_machine_token_type::cancel: //(starting) --cancel-> (cancelled)
                        state = i2c_async_state_machine_state::cancelled;
                        on_cancel_internal(token);
                        break;
                    default: //Otherwise, token has no effect
                        break;
                }
            break;
            case i2c_async_state_machine_state::completed: //Completed/failed/cancelled
            case i2c_async_state_machine_state::faulted:
            case i2c_async_state_machine_state::cancelled:
            //State cannot change if already completed in some regard
            break;
            case i2c_async_state_machine_state::invalid: //??
            default:
                state = i2c_async_state_machine_state::invalid;
            break;
        }
    }

    void on_start_internal(const i2c_async_state_machine_token token)
    {
        const auto n = N++;

        xTaskCreate(i2c_task_fn,
            std::format("i2c_task_{}", n).c_str(),
            2048,
            this,
            5,
            &task_handle
        );

        //Mark task as started
        transition({
            .token_type = i2c_async_state_machine_token_type::status_code,
            .maybe_value = ESP_OK
        });
    }

    void on_complete_internal(const i2c_async_state_machine_token token)
    {
        exit_code = token.maybe_value;
    }

    void on_fault_internal(const i2c_async_state_machine_token token)
    {
        exit_code = token.maybe_value;
    }

    void on_cancel_internal(const i2c_async_state_machine_token token)
    {
        exit_code = token.maybe_value;

        //Delete task
        if (task_handle != nullptr)
        {
            vTaskDelete(task_handle);
            task_handle = nullptr;
        }
    }

    static void i2c_task_fn(void* self_ptr)
    {
        const auto self = static_cast<i2c_async_state_machine*>(self_ptr);

        //Register callback that advances state machine as
        //events occur
        i2c_master_register_event_callbacks(
            self->device_handle,
            &self->callbacks,
            self_ptr
        );

        //While the state machine is not at an end state,
        //suspend this task (which will later be resumed
        //in the callback)
        while (!self->has_reached_end_state())
        {
            vTaskSuspend(self->task_handle);
        }
    }

    static bool i2c_on_trans_done_cb(i2c_master_dev_handle_t handle,
        const i2c_master_event_data_t* event_data,
        void* self_ptr)
    {
        const auto self = static_cast<i2c_async_state_machine*>(self_ptr);
        switch (event_data->event)
        {
            case I2C_EVENT_ALIVE:
                break;
            case I2C_EVENT_TIMEOUT:
                self->transition({
                    .token_type = i2c_async_state_machine_token_type::status_code,
                    .maybe_value = ESP_ERR_TIMEOUT
                });
                break;
            case I2C_EVENT_DONE:
                self->transition({
                    .token_type = i2c_async_state_machine_token_type::status_code,
                    .maybe_value = self->exit_code
                });
                break;
            case I2C_EVENT_NACK:
                self->transition({
                    .token_type = i2c_async_state_machine_token_type::status_code,
                    .maybe_value = self->exit_code == ESP_OK
                        ? ESP_FAIL
                        : self->exit_code
                });
                break;
        }

        return false;
    }

    explicit i2c_async_state_machine(i2c_master_dev_handle_t device_handle)
        : device_handle(device_handle)
    {
        callbacks.on_trans_done = &i2c_on_trans_done_cb;
    }
public:
    [[nodiscard]] i2c_async_state_machine_state get_state() const { return state; }

    [[nodiscard]] bool has_reached_end_state() const
    {
        return state == i2c_async_state_machine_state::completed
            || state == i2c_async_state_machine_state::cancelled
            || state == i2c_async_state_machine_state::faulted;
    }

    [[nodiscard]] uint8_t get_exit_code(uint8_t** buffer_out = nullptr,
        uint8_t* buffer_size_out = nullptr) const
    {
        *buffer_out = rw_buffer;
        *buffer_size_out = rw_buffer_size;

        return exit_code;
    }
    
    void run()
    {
        transition({
            .token_type = i2c_async_state_machine_token_type::start
        });
    }

    void cancel()
    {
        transition({
            .token_type = i2c_async_state_machine_token_type::cancel
        });
    }

    ~i2c_async_state_machine()
    {
        //Unregister task
        if (task_handle != nullptr)
        {
            vTaskDelete(task_handle);
            task_handle = nullptr;
        }
    }

    static i2c_async_state_machine* create_async_read(
        i2c_master_dev_handle_t device_handle,
        uint8_t read_buffer[], const size_t read_buffer_size
    )
    {
        const auto task = new i2c_async_state_machine(device_handle);
        task->type = i2c_async_state_machine_op::read;
        task->rw_buffer = read_buffer;
        task->rw_buffer_size = read_buffer_size;

        return task;
    }

    static i2c_async_state_machine* create_async_write(
        i2c_master_dev_handle_t device_handle,
        uint8_t write_buffer[], const size_t write_buffer_size
    )
    {
        const auto task = new i2c_async_state_machine(device_handle);
        task->type = i2c_async_state_machine_op::write;
        task->rw_buffer = write_buffer;
        task->rw_buffer_size = write_buffer_size;

        return task;
    }
};

#endif //AUDIO_CONTROLLER_ASYNC_STATE_MACHINE_H
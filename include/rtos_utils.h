//
// Created by bendstein on 1/18/2026.
//

#ifndef AUDIO_CONTROLLER_RTOS_UTILS_H
#define AUDIO_CONTROLLER_RTOS_UTILS_H
#include <memory>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

class rtos_semaphore_handle
{
    SemaphoreHandle_t semaphore = nullptr;
    bool from_isr = false;
    BaseType_t* higher_priority_task_woken = nullptr;
public:
    rtos_semaphore_handle() = default;
    explicit rtos_semaphore_handle(SemaphoreHandle_t semaphore);

    rtos_semaphore_handle(const rtos_semaphore_handle& other) = delete;
    rtos_semaphore_handle(rtos_semaphore_handle&& other) = delete;
    rtos_semaphore_handle& operator=(const rtos_semaphore_handle& other) = delete;
    rtos_semaphore_handle& operator=(rtos_semaphore_handle&& other) = delete;

    ~rtos_semaphore_handle();

    static std::unique_ptr<rtos_semaphore_handle> try_take(SemaphoreHandle_t semaphore, TickType_t timeout = portMAX_DELAY);
    static std::unique_ptr<rtos_semaphore_handle> try_take_from_isr(SemaphoreHandle_t semaphore, BaseType_t* higher_priority_task_woken);
};

#endif //AUDIO_CONTROLLER_RTOS_UTILS_H
//
// Created by bendstein on 1/18/2026.
//

#ifndef AUDIO_CONTROLLER_RTOS_UTILS_H
#define AUDIO_CONTROLLER_RTOS_UTILS_H
#include <format>
#include <memory>
#include <portmacro.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#include "app_common.h"

class rtos_semaphore_handle
{
    SemaphoreHandle_t semaphore = nullptr;
    bool from_isr = false;
    BaseType_t* higher_priority_task_woken = nullptr;
public:
    rtos_semaphore_handle() : semaphore(nullptr), from_isr(false), higher_priority_task_woken(nullptr) {}
    explicit rtos_semaphore_handle(const SemaphoreHandle_t semaphore) : rtos_semaphore_handle()
    {
        if (const auto take_result = xSemaphoreTake(semaphore, portMAX_DELAY);
            take_result != pdPASS)
        { //Shouldn't happen since max delay is set
            throw std::runtime_error("rtos_semaphore_handle: failed to get semaphore");
        }

        this->semaphore = semaphore;
    }

    rtos_semaphore_handle(const rtos_semaphore_handle& other) = delete;
    rtos_semaphore_handle(rtos_semaphore_handle&& other) = delete;
    rtos_semaphore_handle& operator=(const rtos_semaphore_handle& other) = delete;
    rtos_semaphore_handle& operator=(rtos_semaphore_handle&& other) = delete;

    ~rtos_semaphore_handle()
    {
        if (semaphore != nullptr)
        {
            if (from_isr)
            {
                if (const auto release_result = xSemaphoreGiveFromISR(semaphore, higher_priority_task_woken);
                    release_result != pdPASS)
                {
                    // FLOGE("Failed to release semaphore: 0x{:04X}.", release_result);
                }
            }
            else
            {
                if (const auto release_result = xSemaphoreGive(semaphore);
                    release_result != pdPASS)
                {
                    FLOGE("Failed to release semaphore: 0x{:04X}.", release_result);
                }
            }
        }
    }

    static std::unique_ptr<rtos_semaphore_handle> try_take(const SemaphoreHandle_t semaphore, const TickType_t timeout = portMAX_DELAY)
    {
        if (const auto take_result = xSemaphoreTake(semaphore, timeout);
            take_result != pdPASS)
        {
            FLOGE("Failed to take semaphore: 0x{:04X}.", take_result);
            return nullptr;
        }

        auto handle = std::make_unique<rtos_semaphore_handle>();
        handle->semaphore = semaphore;
        return handle;
    }

    static std::unique_ptr<rtos_semaphore_handle> try_take_from_isr(const SemaphoreHandle_t semaphore, BaseType_t* higher_priority_task_woken)
    {
        if (const auto take_result = xSemaphoreTakeFromISR(semaphore, higher_priority_task_woken);
            take_result != pdPASS)
        {
            // FLOGE("Failed to take semaphore: 0x{:04X}.", take_result);
            return nullptr;
        }

        auto handle = std::make_unique<rtos_semaphore_handle>();
        handle->semaphore = semaphore;
        handle->from_isr = true;
        handle->higher_priority_task_woken = higher_priority_task_woken;
        return handle;
    }
};

#endif //AUDIO_CONTROLLER_RTOS_UTILS_H
//
// Created by bendstein on 10/15/2026.
//
#include <memory>

#include "app_common.h"
#include "rtos_utils.h"

rtos_semaphore_handle::rtos_semaphore_handle(const SemaphoreHandle_t semaphore) : rtos_semaphore_handle()
{
    if (const auto take_result = xSemaphoreTake(semaphore, portMAX_DELAY);
        take_result != pdPASS)
    { //Shouldn't happen since max delay is set
        throw std::runtime_error("rtos_semaphore_handle: failed to get semaphore");
    }

    this->semaphore = semaphore;
}

rtos_semaphore_handle::~rtos_semaphore_handle()
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
            // else
            // {
            //     FLOGI("Released semaphore: 0x{:04X}.", release_result);
            // }
        }
    }
}

std::unique_ptr<rtos_semaphore_handle> rtos_semaphore_handle::try_take(const SemaphoreHandle_t semaphore, const TickType_t timeout)
{
    if (semaphore == nullptr)
    {
        LOGE("Failed to take semaphore: nullptr.");
        return nullptr;
    }

    if (const auto take_result = xSemaphoreTake(semaphore, timeout);
        take_result != pdPASS)
    {
        FLOGE("Failed to take semaphore: 0x{:04X}.", take_result);
        return nullptr;
    }

    // FLOGI("Took semaphore.");

    auto handle = std::make_unique<rtos_semaphore_handle>();
    handle->semaphore = semaphore;
    return handle;
}

std::unique_ptr<rtos_semaphore_handle> rtos_semaphore_handle::try_take_from_isr(const SemaphoreHandle_t semaphore, BaseType_t* higher_priority_task_woken)
{
    if (semaphore == nullptr)
    {
        // LOGE("Failed to take semaphore: nullptr.");
        return nullptr;
    }

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
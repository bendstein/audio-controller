//
// Created by bendstein on 12/20/2025.
//

#ifndef AUDIO_CONTROLLER_I2C_DEVICE_H
#define AUDIO_CONTROLLER_I2C_DEVICE_H
#include <format>
#include <string>

#include "app_common.h"

enum i2c_device_type : uint8_t
{
    NONE = 0,
    GP2Y0E02B = 1,
    UNKNOWN = 0xFF
};

static std::string i2c_device_type_name(const i2c_device_type type)
{
    switch (type)
    {
        case GP2Y0E02B:
            return NAMEOF(GP2Y0E02B);
        case NONE:
            return NAMEOF(NONE);
        case UNKNOWN:
        default:
            return NAMEOF(UNKNOWN);
    }
}

struct i2c_device
{
    i2c_master_dev_handle_t handle;
    uint8_t address;
    i2c_device_type type;

    [[nodiscard]]
    std::string to_string() const
    {
        return std::format(
            "<{}|0x{:02X} (0x{:08X})>",
            i2c_device_type_name(this->type),
            this->address,
            reinterpret_cast<uintptr_t>(this->handle)
        );
    }
};

#endif //AUDIO_CONTROLLER_I2C_DEVICE_H
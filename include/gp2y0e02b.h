//
// Created by bendstein on 12/20/2025.
//

#ifndef AUDIO_CONTROLLER_GP2Y0E02B_H
#define AUDIO_CONTROLLER_GP2Y0E02B_H
#include "i2c.h"

#define GP2Y0E02B_I2C_ADDR_DFT 0x80
#define GP2Y0E02B_PIN_ENABLE_VPP gpio_num_t::GPIO_NUM_14

//Addr for read is top 7 bits, with LSB 1
#define GP2Y0E02B_ADDR_AS_READ(addr) (((addr) & 0xFE) | 0x01)

//Addr for write is top 7 bits, with LSB 0
#define GP2Y0E02B_ADDR_AS_WRITE(addr) ((addr) & 0xFE)

namespace gp2y0e02b
{
    void set_i2c_addr(i2c_master_bus_handle_t bus, uint8_t addr_new);

    [[nodiscard]]
    bool ping(const i2c_device* device, int timeout_ms);

    [[nodiscard]]
    std::optional<uint8_t> read_from_register(const i2c_device* device, uint8_t reg, int timeout_ms);
}

#endif //AUDIO_CONTROLLER_GP2Y0E02B_H
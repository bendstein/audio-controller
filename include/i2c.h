//
// Created by bendstein on 12/3/2025.
//

#ifndef AUDIO_CONTROLLER_I2C_H
#define AUDIO_CONTROLLER_I2C_H

#define I2C_PIN_SCL                 gpio_num_t::GPIO_NUM_22
#define I2C_PIN_SDA_0               gpio_num_t::GPIO_NUM_23
#define I2C_PIN_SDA_1               gpio_num_t::GPIO_NUM_14
#define I2C_BUS_PORT_0              (-1)
#define I2C_BUS_GLITCH_CT           7
#define I2C_BUS_INTERRUPT_PRIORITY  0
#define I2C_BUS_TRANS_QUEUE_DEPTH   0
#define I2C_BUS_INTERNAL_PULLUP     1
#define I2C_BUS_ALLOW_SLEEP         0
#define I2C_DEVICE_SCL_SPEED_HZ     100000
#define I2C_DEVICE_SCL_WAIT_US      0

#include "driver/i2c_types.h"

[[nodiscard]]
i2c_master_bus_handle_t i2c_init_bus();

#endif //AUDIO_CONTROLLER_I2C_H
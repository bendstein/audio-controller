//
// Created by bendstein on 12/3/2025.
//

#ifndef AUDIO_CONTROLLER_I2C_H
#define AUDIO_CONTROLLER_I2C_H

constexpr auto I2C_STANDARD_HZ = 100000;
constexpr auto I2C_FAST_HZ = 400000;
constexpr auto I2C_HIGH_SPEED_HZ = 34000000;

constexpr auto I2C_PIN_SCL_0 = GPIO_NUM_22;
constexpr auto I2C_PIN_SDA_0 = GPIO_NUM_23;
constexpr auto I2C_BUS_PORT_0  = -1;

constexpr auto I2C_PIN_SCL_1 = GPIO_NUM_32;
constexpr auto I2C_PIN_SDA_1 = GPIO_NUM_14;
constexpr auto I2C_BUS_PORT_1 = -1;

constexpr auto I2C_BUS_GLITCH_CT = 7;
constexpr auto I2C_BUS_INTERRUPT_PRIORITY = 0;
constexpr auto I2C_BUS_TRANS_QUEUE_DEPTH = 0;
constexpr auto I2C_BUS_INTERNAL_PULLUP = 0;
constexpr auto I2C_BUS_ALLOW_SLEEP = 0;
constexpr auto I2C_DEV_SCL_WAIT_US = 0;

#endif //AUDIO_CONTROLLER_I2C_H
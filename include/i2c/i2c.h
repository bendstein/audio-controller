//
// Created by bendstein on 12/3/2025.
//

#ifndef AUDIO_CONTROLLER_I2C_H
#define AUDIO_CONTROLLER_I2C_H

#define I2C_STANDARD_HZ   100000
#define I2C_FAST_HZ       400000

#define I2C_PIN_SCL_0               gpio_num_t::GPIO_NUM_22
#define I2C_PIN_SDA_0               gpio_num_t::GPIO_NUM_23
#define I2C_BUS_PORT_0              (-1)

#define I2C_PIN_SCL_1               gpio_num_t::GPIO_NUM_32
#define I2C_PIN_SDA_1               gpio_num_t::GPIO_NUM_14
#define I2C_BUS_PORT_1              (-1)

#define I2C_BUS_GLITCH_CT           7
#define I2C_BUS_INTERRUPT_PRIORITY  0
#define I2C_BUS_TRANS_QUEUE_DEPTH   0
#define I2C_BUS_INTERNAL_PULLUP     0
#define I2C_BUS_ALLOW_SLEEP         0
#define I2C_DEV_IS_FAST             true
#define I2C_DEV_SCL_WAIT_US         0

#define I2C_DEV_SCL_SPEED_HZ        (I2C_DEV_IS_FAST ? I2C_FAST_HZ : I2C_STANDARD_HZ)

#endif //AUDIO_CONTROLLER_I2C_H
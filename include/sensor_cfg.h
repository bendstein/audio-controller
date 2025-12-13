//
// Created by bendstein on 12/4/2025.
//

#ifndef AUDIO_CONTROLLER_SENSOR_CFG_H
#define AUDIO_CONTROLLER_SENSOR_CFG_H
#include "i2c.h"
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>

#define GP2Y0E02B_I2C_ADDR_DFT 0x80
#define PIN_ENABLE_VPP  gpio_num_t::GPIO_NUM_14
#define portTICK_PERIOD_US ((TickType_t)1000 / portTICK_PERIOD_MS)

void set_gp2y0e02b_i2c_addr(i2c_master_bus_handle_t bus, uint8_t addr_new);

#endif //AUDIO_CONTROLLER_SENSOR_CFG_H
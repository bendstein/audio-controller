//
// Created by bendstein on 12/1/2025.
//

#ifndef AUDIO_CONTROLLER_MAIN_H
#define AUDIO_CONTROLLER_MAIN_H
#include "i2c.h"

[[noreturn]]
[[maybe_unused]]
void configure_gp2y0e02b(i2c_master_bus_handle_t bus);

#endif //AUDIO_CONTROLLER_MAIN_H
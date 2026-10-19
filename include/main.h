//
// Created by bendstein on 1/10/2026.
//

#ifndef AUDIO_CONTROLLER_MAIN_H
#define AUDIO_CONTROLLER_MAIN_H
#include "app_common.h"

// #define CFG_GP2Y0E02B_I2C_ADDR (0x10)

[[noreturn]]
void main_controller();

[[noreturn]]
void main_configure_gp2y0e02b();

[[noreturn]]
void main_audio();

extern "C" {
    [[noreturn]]
    void app_main(void);
}

[[noreturn]]
void log_system_state_task(void*);

enum struct main_method_type
{
    controller,
    configure_gp2y0e02b,
    audio
};

constexpr const char* main_method_type_names[] = {
    NAMEOF(main_method_type::controller),
    NAMEOF(main_method_type::configure_gp2y0e02b),
    NAMEOF(main_method_type::audio)
};

//Which mode the application is running in
#ifdef MAIN_METHOD_TYPE_CFG_GP2Y0E02B
constexpr auto MAIN_METHOD_TYPE = main_method_type::configure_gp2y0e02b;
#elifdef MAIN_METHOD_TYPE_AUDIO
constexpr auto MAIN_METHOD_TYPE = main_method_type::audio;
#else
constexpr auto MAIN_METHOD_TYPE = main_method_type::controller;
#endif

constexpr auto MAIN_METHOD_TYPE_NAME = main_method_type_names[static_cast<uint32_t>(MAIN_METHOD_TYPE)];

#endif //AUDIO_CONTROLLER_MAIN_H
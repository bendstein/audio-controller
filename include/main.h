//
// Created by bendstein on 1/10/2026.
//

#ifndef AUDIO_CONTROLLER_MAIN_H
#define AUDIO_CONTROLLER_MAIN_H

// #define CFG_GP2Y0E02B_I2C_ADDR (0x10)

[[noreturn]]
void main_standard();

[[noreturn]]
void main_configure_gp2y0e02b();

extern "C" {
    [[noreturn]]
    void app_main(void);
}

[[noreturn]]
void log_system_state_task(void*);

#endif //AUDIO_CONTROLLER_MAIN_H
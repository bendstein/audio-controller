//
// Created by bendstein on 9/30/2025.
//

#ifndef AUDIO_CONTROLLER_MAIN_H
#define AUDIO_CONTROLLER_MAIN_H

#include <atomic>
#include "common.h"
#include "frequency_range_data.h"

#define PIN_OUT_DEBUG LED_BUILTIN               //Output pin indicating whether debugging
#define PIN_IN_TOGGLE_DEBUG GPIO_NUM_15         //Input pin for toggle debug button

#define PIN_IN_ROT_CYCLE_RATE_A GPIO_NUM_14     //Input pin A for cycle-rate rotary encoder
#define PIN_IN_ROT_CYCLE_RATE_B GPIO_NUM_32     //Input pin B for cycle-rate rotary encoder

#define PIN_OUT_SENSOR_SELECT_0 GPIO_NUM_12     //Sensor select bit 0
#define PIN_OUT_SENSOR_SELECT_1 GPIO_NUM_27     //Sensor select bit 1
#define PIN_OUT_SENSOR_SELECT_2 GPIO_NUM_33     //Sensor select bit 2
#define PIN_IN_SENSOR A3                        //Sensor data

#define PIN_IN_AUDIO_VOLUME A2                  //Input pin to adjust audio volume
#define PIN_OUT_AUDIO_0 A1                      //Pin corresponding to first audio output
#define PIN_OUT_AUDIO_1 A0                      //Pin corresponding to second audio output
#define AUDIO_CHANNEL_0 DAC1                    //DAC channel for PIN_OUT_AUDIO_0
#define AUDIO_CHANNEL_1 DAC2                    //DAC channel for PIN_OUT_AUDIO_1

#define TIMER_INTERVAL 50000                    //Frequency at which update timer should trigger

/**
 * Get the next audio value to output
 * @param t The current time in microseconds
 * @param scale Coefficient to scale the output wave, to control volume
 * @param chord The component frequencies, in hz, of the wave
 * @param chord_len The length of the chord array
 * @return The next value in the output wave, from 0 to 255
 */
uint8_t wave(uint64_t t, double scale, const double chord[], uint8_t chord_len);

/**
 * Get the new frequency for a given sensor
 * @param sensor The index of the sensor
 * @param frequency_range The range of potential output frequencies
 * @return The frequency corresponding to the given sensor value
 */
double calculate_frequency(int sensor, const FrequencyRangeData &frequency_range);

/**
 * Update current frequency values based on sensors
 */
void update_frequency();


/**
 * Callback for timer interrupt
 */
void isr_timer();

/**
 * Callback for debug toggle button interrupt
 */
void isr_debug_toggle();

/**
 * Callback for
 * @param clockwise Whether encoder turned clockwise or counter-clockwise
 */
void isr_rotary_encoder(bool clockwise);

/**
 * Print a string containing general info on current state, if debugging
 */
void print_debug_info();

#endif //AUDIO_CONTROLLER_MAIN_H
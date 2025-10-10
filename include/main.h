//
// Created by bendstein on 9/30/2025.
//

#ifndef AUDIO_CONTROLLER_MAIN_H
#define AUDIO_CONTROLLER_MAIN_H

#include <atomic>
#include "frequency_range_data.h"

#define PIN_IN_DEBUG GPIO_NUM_17            //Input pin to indicate debugging
#define PIN_IN_TOF_0 A2                     //Input pin for first TOF sensor
#define PIN_IN_TOF_1 A3                     //Input pin for second TOF sensor
#define PIN_IN_ZERO A6                      //Input pin to zero sensors
#define PIN_IN_CALIBRATE A7                 //Input pin to calibrate range for sensors
#define PIN_IN_VOLUME A10                   //Input pin to adjust audio volume

#define PIN_OUT_AUDIO_0 A1                  //Pin corresponding to first audio output
#define PIN_OUT_AUDIO_1 A0                  //Pin corresponding to second audio output
#define PIN_OUT_INDICATE_ZERO A5            //Output pin indicating whether currently zero-ing sensors
#define PIN_OUT_INDICATE_CAL GPIO_NUM_21    //Output pin indicating whether currently calibrating sensors

#define AUDIO_CHANNEL_0 DAC1                //DAC channel for PIN_OUT_AUDIO_0
#define AUDIO_CHANNEL_1 DAC2                //DAC channel for PIN_OUT_AUDIO_1

#define BUFFER_ZERO 48                      //Add buffer when zeroing to help account for noise
#define TIMER_INTERVAL 50000                //Frequency at which update timer should trigger

/**
 * Whether sensors are being calibrated, and if so,
 * which type
 */
enum CalibrationState
{
    None = 0,
    Zeroing = 1,
    Calibrating = 2
};

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
 * Handle calibrating sensors, as well as
 * changes in calibration state.
 * @param toggle_state The calibration type to toggle
 * @return The new calibration state
 */
CalibrationState handle_calibration(CalibrationState toggle_state);

/**
 * Callback for timer interrupt
 */
void isr_timer();

/**
 * Callback for zero button interrupt
 */
void isr_zero();

/**
 * Callback for calibrate button interrupt
 */
void isr_calibrate();

/**
 * Callback for debugging switch interrupt
 */
void isr_debug();

/**
 * Print a string containing general info on current state, if debugging
 */
void print_debug_info();

#endif //AUDIO_CONTROLLER_MAIN_H
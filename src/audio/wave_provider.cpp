//
// Created by bendstein on 1/2/2026.
//
#include "audio/wave_provider.h"

#include <cmath>

float wave_provider::wave(const long long time_us, const float tones[], const size_t tones_length) const { return 0; }

float sin_wave_provider::wave(const long long time_us, const float tones[], size_t const tones_length) const {
    float sum = 0;
    size_t tones_used = 0;

    //Sum up component sine waves at this time
    for(auto i = 0; i < tones_length; i++) {
        //Skip frequencies of 0
        if(const auto frequency = tones[i]; frequency > 0) {
            //Put time_us within period to handle long long.
            //Can narrow type to float since frequency is a float, and result of
            //modulus should never exceed it.
            const auto period = 1 / frequency;
            const auto time_us_adj = static_cast<float>(std::fmodl(time_us, period));

            // logi(NAMEOF(sin_wave), std::format("{} -> {} / {}MHz", time_us, time_us_adj, frequency));

            //Sine wave at this frequency, transformed from y=[-1,1] to y=[0,1]
            const auto component = (1 + sin(2 * M_PI * frequency * time_us_adj)) / 2.;
            sum += component;
            tones_used++;
        }
    }

    //Normalize amplitude based on number of frequencies in chord so that range is [0,1]
    return tones_used == 0? 0 : sum / tones_used;
}

float square_wave_provider::wave(const long long time_us, const float tones[], const size_t tones_length) const {
    float sum = 0;
    size_t tones_used = 0;

    for(auto i = 0; i < tones_length; i++) {
        //Skip frequencies of 0
        if(const auto frequency = tones[i]; frequency > 0) {
            //Put time_us within period to handle long long.
            //Can narrow type to float since frequency is a float, and result of
            //modulus should never exceed it.
            const auto period = 1 / frequency;
            const auto time_us_adj = static_cast<float>(std::fmodl(time_us, period));

            //If ratio into period is <= duty cycle, 1, else 0
            const auto component = time_us_adj * frequency <= duty_cycle? 1 : 0;
            sum += component;
            tones_used++;
        }
    }

    //Normalize amplitude based on number of frequencies in chord so that range is [0,1]
    return tones_used == 0? 0 : sum / tones_used;
}

float sawtooth_wave_provider::wave(const long long time_us, const float tones[], const size_t tones_length) const {
    float sum = 0;
    size_t tones_used = 0;

    //Sum up component square waves at this time
    for(auto i = 0; i < tones_length; i++) {
        //Skip frequencies of 0
        if(const auto frequency = tones[i]; frequency > 0) {
            //Put time_us within period to handle long long.
            //Can narrow type to float since frequency is a float, and result of
            //modulus should never exceed it.
            const auto period = 1 / frequency;
            const auto time_us_adj = static_cast<float>(std::fmodl(time_us, period));

            //Ratio of current time to period (accounting for duty cycle).
            //If ratio is past duty cycle, 0.
            const auto ratio = time_us_adj * frequency;
            const auto component = ratio <= duty_cycle
                ? time_us_adj * frequency / duty_cycle
                : 0;

            sum += component;
            tones_used++;
        }
    }

    //Normalize amplitude based on number of frequencies in chord so that range is [0,1]
    return tones_used == 0? 0 : sum / tones_used;
}

float triangle_wave_provider::wave(const long long time_us, const float tones[], const size_t tones_length) const {
    float sum = 0;
    size_t tones_used = 0;

    //Sum up component square waves at this time
    for(auto i = 0; i < tones_length; i++) {
        //Skip frequencies of 0
        if(const auto frequency = tones[i]; frequency > 0) {
            //Put time_us within period to handle long long.
            //Can narrow type to float since frequency is a float, and result of
            //modulus should never exceed it.
            const auto period = 1 / frequency;
            const auto time_us_adj = static_cast<float>(std::fmodl(time_us, period));

            //Ratio of current time to half of period (accounting for duty cycle). Invert when past peak
            //If ratio is past duty cycle, 0.
            const auto ratio = time_us_adj * frequency;
            const auto component_0 = ratio <= duty_cycle
                ? time_us_adj * frequency * 2 / duty_cycle
                : 0;
            const auto component = component_0 > 1? (1 - (component_0 - 1)) : component_0;

            sum += component;
            tones_used++;
        }
    }

    //Normalize amplitude based on number of frequencies in chord so that range is [0,1]
    return tones_used == 0? 0 : sum / tones_used;
}
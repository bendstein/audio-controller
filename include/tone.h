//
// Created by bendstein on 12/2/2025.
//

#ifndef AUDIO_CONTROLLER_TONE_H
#define AUDIO_CONTROLLER_TONE_H

#include "cmath"
#include "app_common.h"

/**
 * Get the next (DAC) audio value to output
 * @param time_micro Elapsed time in microseconds
 * @param scale Coefficient to scale the output wave, to control volume
 * @param chord The component frequencies, in hz, of the wave
 * @param chord_len The length of chord array
 * @return The DAC level to write
 */
typedef uint16_t WaveProvider(uint64_t time_micro, double scale, const double chord[], uint8_t chord_len);

/**
 * Class that handles outputting audio to DAC
 */
class Tone
{
    WaveProvider* wave_provider = nullptr;
    uint16_t dac_level = 0;
public:
    Tone() { }

    explicit Tone(WaveProvider* wave_provider) : wave_provider(wave_provider) { }

    /**
     * Calculate next DAC value
     * @param time_micro Elapsed time in microseconds
     * @param scale Coefficient to scale the output wave, to control volume
     * @param chord The component frequencies, in hz, of the wave
     * @param chord_len The length of chord array
     */
    void Update(const uint64_t time_micro, const double scale, const double chord[], const uint8_t chord_len)
    {
        if (wave_provider == nullptr)
            return;

        dac_level = wave_provider(time_micro, scale, chord, chord_len);
    }

    /**
     * Output current value to DAC
     */
    void Play() const
    {
        //TODO

    }

    /**
     * Calculate and output next DAC value
     * @param time_micro Elapsed time in microseconds
     * @param scale Coefficient to scale the output wave, to control volume
     * @param chord The component frequencies, in hz, of the wave
     * @param chord_len The length of chord array
     */
    void UpdateAndPlay(const uint64_t time_micro, const double scale, const double chord[], const uint8_t chord_len)
    {
        Update(time_micro, scale, chord, chord_len);
        Play();
    }

    /**
     * Default implementation of WaveProvider which uses a sine wave
     * @param time_micro Elapsed time in microseconds
     * @param scale Coefficient to scale the output wave, to control volume
     * @param chord The component frequencies, in hz, of the wave
     * @param chord_len The length of chord array
     * @return The DAC level to write
     */
    static uint16_t DefaultWaveProvider_Sine(const uint64_t time_micro, const double scale, const double chord[], const uint8_t chord_len)
    {
        //microseconds => seconds
        const auto ts = static_cast<long double>(time_micro) / 1000000;

        //Sum up chord components
        long double f = 0;
        uint8_t count = 0;

        for (auto i = 0; chord != nullptr && i < chord_len; i++)
        {
            const auto chord_value = chord[i];

            //Ignore frequencies below threshold
            if (chord_value <= FREQ_MIN_HZ)
                continue;

            //Calculate sine wave for this frequency and time, scaled for volume
            const auto value = scale * sin(2 * M_PI * chord_value * ts);
            f += value;
            count += 1;
        }

        //Translate the waves to be centered at scale instead of at 0,
        //to prevent negative values
        if (count > 0)
        {
            f = scale + f / count;
        }
        //Return 0 if no frequencies were considered
        else
        {
            return 0;
        }

        //Map output value to a value between 0 and 255
        const auto v = static_cast<int32_t>(std::floor((DAC_MAX / 2.) * f));

        if (v > DAC_MAX)
            return DAC_MAX;

        if (v < 0)
            return 0;

        return static_cast<uint16_t>(v);
    }
};

#endif //AUDIO_CONTROLLER_TONE_H
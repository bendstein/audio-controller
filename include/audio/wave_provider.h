//
// Created by bendstein on 1/2/2026.
//

#ifndef AUDIO_CONTROLLER_WAVE_PROVIDER_H
#define AUDIO_CONTROLLER_WAVE_PROVIDER_H

#include <cstddef>
#include <cassert>

class wave_provider
{
public:
    wave_provider() = default;
    virtual ~wave_provider() = default;
    [[nodiscard]] virtual float wave(long long time_us, const float tones[], size_t tones_length) const;
};

class sin_wave_provider final : public wave_provider
{
public:
    sin_wave_provider() = default;
    float wave(long long time_us, const float tones[], size_t tones_length) const override;

    ~sin_wave_provider() override = default;
};

class square_wave_provider final : public wave_provider
{
    const float duty_cycle = 0.5;
public:
    square_wave_provider() = default;
    explicit square_wave_provider(const float duty_cycle) : duty_cycle(duty_cycle)
    {
        assert(duty_cycle >= 0 && duty_cycle <= 1);
    }

    [[nodiscard]] float get_duty_cycle() const { return duty_cycle; }

    float wave(long long time_us, const float tones[], size_t tones_length) const override;

    ~square_wave_provider() override = default;
};

class sawtooth_wave_provider final : public wave_provider
{
    const float duty_cycle = 1;
public:
    sawtooth_wave_provider() = default;
    explicit sawtooth_wave_provider(const float duty_cycle) : duty_cycle(duty_cycle)
    {
        assert(duty_cycle >= 0 && duty_cycle <= 1);
    }

    [[nodiscard]] float get_duty_cycle() const { return duty_cycle; }

    float wave(long long time_us, const float tones[], size_t tones_length) const override;

    ~sawtooth_wave_provider() override = default;
};

class triangle_wave_provider final : public wave_provider
{
    const float duty_cycle = 1;
public:
    triangle_wave_provider() = default;
    explicit triangle_wave_provider(const float duty_cycle) : duty_cycle(duty_cycle)
    {
        assert(duty_cycle > 0 && duty_cycle < 1);
    }

    [[nodiscard]] float get_duty_cycle() const { return duty_cycle; }

    float wave(long long time_us, const float tones[], size_t tones_length) const override;

    ~triangle_wave_provider() override = default;
};

#endif //AUDIO_CONTROLLER_WAVE_PROVIDER_H
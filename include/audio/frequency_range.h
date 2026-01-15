//
// Created by bendstein on 1/2/2026.
//

#ifndef AUDIO_CONTROLLER_FREQUENCY_RANGE_H
#define AUDIO_CONTROLLER_FREQUENCY_RANGE_H
#include <functional>
#include <memory>
#include <utility>

#include "notes.h"

class frequency_range
{
protected:
    frequency_range(frequency_range const&) = default;
    frequency_range(frequency_range&&) = default;
    frequency_range& operator=(frequency_range const&) = default;
    frequency_range& operator=(frequency_range&&) = default;
public:
    frequency_range() = default;
    virtual ~frequency_range() = default;
    [[nodiscard]] float get_tone_hz(const float value) const { return get_tone(value) * US_PER_SECOND; }
    [[nodiscard]] virtual std::unique_ptr<frequency_range> clone() const = 0;
    [[nodiscard]] virtual float get_tone(float value) const = 0;
};

class single_frequency_range final : public frequency_range
{
    const tone t;
public:
    explicit single_frequency_range(const tone& t) : t(t) {}

    [[nodiscard]] std::unique_ptr<frequency_range> clone() const override { return std::make_unique<single_frequency_range>(*this); }
    [[nodiscard]] float get_tone(float value) const override { return t.frequency_megahz(); }
};

class continuous_frequency_range final : public frequency_range
{
    typedef std::function<float(float)> scale_fn;
    static float scale_fn_none(float) { return 0; }

    const tone min {};
    const tone max {};
    const scale_fn scale;
public:
    continuous_frequency_range(const tone& min, const tone& max, scale_fn  scale)
        : min(min), max(max), scale(std::move(scale)) {}

    [[nodiscard]] std::unique_ptr<frequency_range> clone() const override { return std::make_unique<continuous_frequency_range>(*this); }

    [[nodiscard]] float get_tone(float value) const override;

    [[nodiscard]] float map_to_range(const float value) const
    {
        const auto min_freq = min.frequency_hz();
        const auto max_freq = max.frequency_hz();
        const auto value_scaled = scale(value);
        return min_freq + value_scaled * (max_freq - min_freq);
    }

    static float scale_fn_linear(const float value) { return value; }

    static std::function<float(float)> create_scale_fn_log(const float log_base)
    {
        //(log_base - 1) * value + 1 keeps w/in [0, 1], division is for change of base
        return [=](const float f) { return logf((log_base - 1) * f + 1) / logf(log_base); };
    }

    static std::function<float(float)> create_scale_fn_power(const float power)
    {
        return [=](const float f) { return powf(f, power); };
    }

    static std::function<float(float)> create_scale_fn_exp(const float exp_base)
    {
        //-1 and division are to adjust to [0, 1]
        return [=](const float f) { return (powf(exp_base, f) - 1) / (exp_base - 1); };
    }
};

class piecewise_frequency_range_breakpoint
{
    float breakpoint;
    std::unique_ptr<frequency_range> range;
public:
    piecewise_frequency_range_breakpoint(const float breakpoint, const frequency_range& range)
        : breakpoint(breakpoint), range(range.clone()) {}

    [[nodiscard]] static float get_tone(const piecewise_frequency_range_breakpoint breakpoints[], size_t breakpoints_len, float value);
    [[nodiscard]] static float get_tone_hz(const piecewise_frequency_range_breakpoint breakpoints[], const size_t breakpoints_len, const float value)
    {
        return get_tone(breakpoints, breakpoints_len, value) * US_PER_SECOND;
    }
};

#endif //AUDIO_CONTROLLER_FREQUENCY_RANGE_H
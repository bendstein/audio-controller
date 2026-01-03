//
// Created by bendstein on 1/2/2026.
//

#ifndef AUDIO_CONTROLLER_FREQUENCY_RANGE_H
#define AUDIO_CONTROLLER_FREQUENCY_RANGE_H
#include <optional>

#include "notes.h"

class frequency_range
{
public:
    frequency_range() = default;
    virtual ~frequency_range() = default;
    [[nodiscard]] virtual std::optional<const tone*> get_tone(double value) const { return new tone(); }
};

class single_frequency_range final : public frequency_range
{
    const tone* t {};
public:
    single_frequency_range() = default;
    explicit single_frequency_range(const tone* t) : t(t) {}

    ~single_frequency_range() override = default;

    [[nodiscard]] std::optional<const tone*> get_tone(double value) const override { return t; }
};

class continuous_frequency_range : public frequency_range
{
    const tone* min {};
    const tone* max {};
public:
    continuous_frequency_range() = default;
    continuous_frequency_range(const tone* min, const tone* max) : min(min), max(max) { }

    ~continuous_frequency_range() override = default;
    [[nodiscard]] std::optional<const tone*> get_tone(double value) const override;

    [[nodiscard]] virtual double map_to_range(const double value) const
    {
        const auto min_freq = min->frequency_hz();
        const auto max_freq = max->frequency_hz();
        return min_freq + value * (max_freq - min_freq);
    }
};

class log_continuous_frequency_range final : public continuous_frequency_range
{
    const double log_base = M_E;
public:
    log_continuous_frequency_range() = default;

    log_continuous_frequency_range(const tone* min, const tone* max) : continuous_frequency_range(min, max) { }
    log_continuous_frequency_range(const tone* min, const tone* max, const double log_base) : continuous_frequency_range(min, max), log_base(log_base) { }

    ~log_continuous_frequency_range() override = default;

    [[nodiscard]] double map_to_range(const double value) const override
    {
        //Map to log scale and then use base impl. (log_base - 1) * value + 1 keeps w/in [0, 1],
        //division is for change of base
        const auto log_scaled = log((log_base - 1) * value + 1) / log(log_base);
        return continuous_frequency_range::map_to_range(log_scaled);
    }
};

class power_continuous_frequency_range final : public continuous_frequency_range
{
    const double power = 2;
public:
    power_continuous_frequency_range() = default;

    power_continuous_frequency_range(const tone* min, const tone* max) : continuous_frequency_range(min, max) { }
    power_continuous_frequency_range(const tone* min, const tone* max, const double power) : continuous_frequency_range(min, max), power(power) { }

    ~power_continuous_frequency_range() override = default;

    [[nodiscard]] double map_to_range(const double value) const override
    {
        //Map to power scale and then use base impl
        return continuous_frequency_range::map_to_range(pow(value, power));
    }
};

class exp_continuous_frequency_range final : public continuous_frequency_range
{
    const double exp = M_E;
public:
    exp_continuous_frequency_range() = default;

    exp_continuous_frequency_range(const tone* min, const tone* max) : continuous_frequency_range(min, max) { }
    exp_continuous_frequency_range(const tone* min, const tone* max, const double exp) : continuous_frequency_range(min, max), exp(exp) { }

    ~exp_continuous_frequency_range() override = default;

    [[nodiscard]] double map_to_range(const double value) const override
    {
        //Map to exponential scale and then use base impl. -1 and division
        //are to adjust to [0, 1]
        return continuous_frequency_range::map_to_range(
            (pow(exp, value) - 1) / (exp - 1)
        );
    }
};

struct piecewise_frequency_range_breakpoint
{
    const double breakpoint = 0;
    const frequency_range* range {};
};

class piecewise_frequency_range final : public frequency_range
{
    const piecewise_frequency_range_breakpoint* breakpoints;
    const size_t breakpoints_len;
public:
    piecewise_frequency_range() : breakpoints(nullptr), breakpoints_len(0) { }
    piecewise_frequency_range(const piecewise_frequency_range_breakpoint* breakpoints, const size_t breakpoints_len) : breakpoints(breakpoints), breakpoints_len(breakpoints_len) {}

    ~piecewise_frequency_range() override = default;
    [[nodiscard]] std::optional<const tone*> get_tone(double value) const override;
};

#endif //AUDIO_CONTROLLER_FREQUENCY_RANGE_H
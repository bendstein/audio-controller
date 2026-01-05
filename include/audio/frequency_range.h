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
public:
    frequency_range() = default;
    frequency_range(frequency_range const&) = default;
    frequency_range(frequency_range&&) = default;
    frequency_range& operator=(frequency_range const&) = default;
    frequency_range& operator=(frequency_range&&) = default;
    virtual ~frequency_range() = default;

    [[nodiscard]] virtual float get_tone(float value) const { return 0; }
};

class single_frequency_range final : public frequency_range
{
    std::unique_ptr<tone> t;
public:
    //By default assign ownership to a new copy of the default tone
    single_frequency_range() : t(std::make_unique<tone>(*tone::dft())) {}

    //Assign ownership to a copy of the other tone
    single_frequency_range(single_frequency_range const& other) : t(std::make_unique<tone>(*other.t)) {}

    //Transfer ownership of the other tone to this. Bc the other needs to remain valid, give it ownership to a new copy of the default tone
    single_frequency_range(single_frequency_range&& other) noexcept
        : t(std::exchange(other.t, std::make_unique<tone>(*tone::dft()))) {}

    //Assign ownership to a copy of the other tone
    single_frequency_range& operator=(single_frequency_range const& other)
    {
        if (&other == this)
            return *this;

        //Do swap with a new unique ptr so the old value is deallocated
        auto tmp = std::make_unique<tone>(*other.t);
        t.swap(tmp);

        return *this;
    }

    //Transfer ownership of the other tone to this. Bc the other needs to remain valid, give it ownership to a new copy of the default tone.
    single_frequency_range& operator=(single_frequency_range&& other) noexcept
    {
        if (&other == this)
            return *this;

        t = std::exchange(other.t, std::make_unique<tone>(tone(*tone::dft())));
        return *this;
    }

    explicit single_frequency_range(const tone& t) : t(std::make_unique<tone>(t)) {}

    ~single_frequency_range() override = default;

    [[nodiscard]] float get_tone(float value) const override { return t->frequency_megahz(); }
};

class continuous_frequency_range final : public frequency_range
{
    typedef std::function<float(float)> scale_fn;
    static float scale_fn_none(float) { return 0; }

    std::unique_ptr<tone> min {};
    std::unique_ptr<tone> max {};
    std::unique_ptr<scale_fn> scale;
public:
    //By default assign ownership to new copies of the default tone, scale fn
    continuous_frequency_range()
        : min(std::make_unique<tone>(*tone::dft())),
          max(std::make_unique<tone>(*tone::dft())),
          scale(std::make_unique<scale_fn>(scale_fn_none)) {}

    //Assign ownership to new copies of the other's tones, scale fn
    continuous_frequency_range(continuous_frequency_range const& other)
        : min(std::make_unique<tone>(*other.min)),
          max(std::make_unique<tone>(*other.max)),
          scale(std::make_unique<scale_fn>(*other.scale)) {}

    //Transfer ownership of the other's tones to this. Bc the other needs to remain valid, give it ownership to new copies of the default tone.
    //Same with scale fn
    continuous_frequency_range(continuous_frequency_range&& other) noexcept
        : min(std::exchange(other.min, std::make_unique<tone>(*tone::dft()))),
          max(std::exchange(other.max, std::make_unique<tone>(*tone::dft()))),
          scale(std::exchange(other.scale, std::make_unique<scale_fn>(scale_fn_none))) {}

    //Assign ownership to new copies of the other's tones, scale fn
    continuous_frequency_range& operator=(continuous_frequency_range const& other)
    {
        if (&other == this)
            return *this;

        //Do swap with new unique ptrs so the old values are deallocated
        auto tmp_min = std::make_unique<tone>(*other.min);
        auto tmp_max = std::make_unique<tone>(*other.max);
        auto tmp_scale = std::make_unique<scale_fn>(*other.scale);
        min.swap(tmp_min);
        max.swap(tmp_max);
        scale.swap(tmp_scale);

        return *this;
    }

    //Transfer ownership of the other's tones to this. Bc the other needs to remain valid, give it ownership to new copies of the default tone.
    //Same w/ fn pointer
    continuous_frequency_range& operator=(continuous_frequency_range&& other) noexcept
    {
        if (&other == this)
            return *this;

        min = std::exchange(other.min, std::make_unique<tone>(*tone::dft()));
        max = std::exchange(other.max, std::make_unique<tone>(*tone::dft()));
        scale = std::exchange(other.scale, std::make_unique<scale_fn>(scale_fn_none));
        return *this;
    }

    continuous_frequency_range(const tone& min, const tone& max, const scale_fn& scale)
        : min(std::make_unique<tone>(min)),
          max(std::make_unique<tone>(max)),
          scale(std::make_unique<scale_fn>(scale)) { }

    ~continuous_frequency_range() override = default;

    [[nodiscard]] float get_tone(float value) const override;

    [[nodiscard]] float map_to_range(const float value) const
    {
        const auto min_freq = min->frequency_hz();
        const auto max_freq = max->frequency_hz();
        const auto value_scaled = (*scale)(value);
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
    //By default assign ownership to new default frequency range
    piecewise_frequency_range_breakpoint()
        : breakpoint(0),
          range(std::make_unique<frequency_range>(frequency_range())) {}

    //Assign ownership to deep copy of other's range, assign breakpoint as normal
    piecewise_frequency_range_breakpoint(piecewise_frequency_range_breakpoint const& other)
        : breakpoint(other.breakpoint),
          range(std::make_unique<frequency_range>(*other.range)) {}

    //Move ownership of other's range, giving other ownership of a new default frequency range.
    //Assign breakpoint as normal
    piecewise_frequency_range_breakpoint(piecewise_frequency_range_breakpoint&& other) noexcept
        : breakpoint(other.breakpoint),
          range(std::exchange(other.range, std::make_unique<frequency_range>(frequency_range()))) {}

    //Assign ownership to deep copy of other's range, assign breakpoint as normal
    piecewise_frequency_range_breakpoint& operator=(piecewise_frequency_range_breakpoint const& other)
    {
        if (&other == this)
            return *this;

        breakpoint = other.breakpoint;

        //Do swap with new unique ptrs so the old values are deallocated
        auto tmp = std::make_unique<frequency_range>(*other.range);
        range.swap(tmp);

        return *this;
    }

    //Move ownership of other's range, giving other ownership of a new default frequency range.
    //Assign breakpoint as normal
    piecewise_frequency_range_breakpoint& operator=(piecewise_frequency_range_breakpoint&& other) noexcept
    {
        if (&other == this)
            return *this;

        breakpoint = other.breakpoint;
        range = std::exchange(other.range, std::make_unique<frequency_range>(frequency_range()));

        return *this;
    }

    ~piecewise_frequency_range_breakpoint() = default;

    [[nodiscard]] static float get_tone(const piecewise_frequency_range_breakpoint breakpoints[], size_t breakpoints_len, float value);
};

#endif //AUDIO_CONTROLLER_FREQUENCY_RANGE_H
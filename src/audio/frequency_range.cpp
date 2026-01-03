#include "audio/frequency_range.h"
//
// Created by bendstein on 1/2/2026.
//

std::optional<const tone*> continuous_frequency_range::get_tone(const double value) const
{
    if (min == nullptr || max == nullptr) //Invalid range/fn
        return std::nullopt;

    const auto min_freq = min->frequency_hz();
    const auto max_freq = max->frequency_hz();

    if (min_freq == max_freq) //Single frequency
        return min;

    //Clamp value to [0, 1]
    if (value <= 0)
        return min;
    if (value >= 1)
        return max;

    return new tone(map_to_range(value));
}

std::optional<const tone*> piecewise_frequency_range::get_tone(const double value) const
{
    if (breakpoints_len > 0 && breakpoints != nullptr)
    {
        double previous_breakpoint = 0;

        for (auto i = 0; i < breakpoints_len; i++)
        {
            const auto [breakpoint, child_range] = breakpoints[i];

            if (breakpoint == previous_breakpoint) //Prevent division by 0
                continue;

            if (value < breakpoint)
            {
                //Adj value to be ratio between previous and current breakpoint so
                //that the child function uses its full range
                const auto adj_value = (value - previous_breakpoint) / (breakpoint - previous_breakpoint);
                return child_range == nullptr? std::nullopt : child_range->get_tone(adj_value);
            }

            previous_breakpoint = breakpoint;
        }
    }

    return std::nullopt; //Invalid set of breakpoints
}

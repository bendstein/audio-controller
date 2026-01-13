//
// Created by bendstein on 1/2/2026.
//
#include "audio/frequency_range.h"

float continuous_frequency_range::get_tone(const float value) const
{
    const auto min_freq = min.frequency_megahz();
    const auto max_freq = max.frequency_megahz();

    if (min_freq == max_freq) //Single frequency
        return min_freq;

    //Clamp value to range
    if (value <= 0)
        return min_freq;
    if (value >= 1)
        return max_freq;

    return map_to_range(value);
}

float piecewise_frequency_range_breakpoint::get_tone(const piecewise_frequency_range_breakpoint breakpoints[], const size_t breakpoints_len, const float value)
{
    if (breakpoints_len > 0 && breakpoints != nullptr)
    {
        float previous_breakpoint = 0;

        for (auto i = 0; i < breakpoints_len; i++)
        {
            const auto breakpoint = breakpoints[i].breakpoint;
            const auto child_range = &*breakpoints[i].range;

            if (breakpoint == previous_breakpoint) //Prevent division by 0
                continue;

            if (value < breakpoint)
            {
                //Adj value to be ratio between previous and current breakpoint so
                //that the child function uses its full range
                const auto adj_value = (value - previous_breakpoint) / (breakpoint - previous_breakpoint);
                return child_range == nullptr? 0 : child_range->get_tone(adj_value);
            }

            previous_breakpoint = breakpoint;
        }
    }

    return 0; //Invalid set of breakpoints
}

//
// Created by bendstein on 1/2/2026.
//
#include "audio/frequency_range.h"

tone continuous_frequency_range::get_tone(const float value) const
{
    if (min.is_equivalent_to(max)) //Single frequency
        return min;

    //Clamp value to range
    if (value <= 0)
        return min;
    if (value >= 1)
        return max;

    return map_to_range(value);
}

tone piecewise_frequency_range_breakpoint::get_tone(const piecewise_frequency_range_breakpoint breakpoints[], const size_t breakpoints_len, const float value)
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

                // logi("get_tone", std::format("val: {}, bp: {}, bp prev: {}, adj: {}, child: 0x{:08X}",
                //     value, breakpoint, previous_breakpoint, adj_value, reinterpret_cast<uintptr_t>(child_range)));

                return child_range == nullptr? *tone::dft() : child_range->get_tone(adj_value);
            }

            previous_breakpoint = breakpoint;
        }
    }

    return *tone::dft(); //Invalid set of breakpoints
}

//
// Created by bendstein on 10/5/2025.
//

#ifndef AUDIO_CONTROLLER_TOF_SENSOR_CALIBRATION_DATA_H
#define AUDIO_CONTROLLER_TOF_SENSOR_CALIBRATION_DATA_H
#include <atomic>
#include <string>

/**
 * Data representing the valid range of values that a sensor
 * can produce.
 */
class SensorRangeData
{
    /**
     * The max value that the sensor produces when not being interacted
     * with.
     */
    std::atomic_uint16_t zero {};

    /**
     * The range of values that the sensor can produce when it is
     * being interacted with. Each is a 16-bit, unsigned integer,
     * with the max as the most-significant 16 bits and the min
     * as the least-significant 16 bits.
     */
    std::atomic_uint32_t minmax {};
public:
    /**
     *
     * @param zero The max value that the sensor produces when not being
     *  interacted with.
     * @param min The min value that the sensor can produce when it is
     *  being interacted with.
     * @param max The max value that the sensor can produce when it is
     *  being interacted with.
     */
    SensorRangeData(const uint16_t zero, const uint16_t min, const uint16_t max)
    {
        this->zero = zero;
        this->minmax = (max << 16) | min;
    }

    /**
     * Default constructor, with all values initialized to 0
     */
    SensorRangeData() : SensorRangeData(0, 0, 0) {}

    /**
     * Copy constructor
     * @param other The SensorRangeData to copy values from
     */
    explicit SensorRangeData(const SensorRangeData* other) : SensorRangeData()
    {
        Update(*other);
    }

    /**
     * @return The sensor's zero
     */
    uint16_t GetZero() const
    {
        return this->zero.load();
    }

    /**
     * @param new_zero The sensor's new zero
     */
    void SetZero(const uint16_t new_zero)
    {
        this->zero.store(new_zero);
    }

    /**
     * @return The sensor's min
     */
    uint16_t GetMin() const
    {
        //Min is the bottom 16 bits of minmax
        return this->minmax.load() & 0xFFFF;
    }

    /**
     * @param new_min The sensor's new min
     */
    void SetMin(const uint16_t new_min)
    {
        //Min is the bottom 16 bits of minmax,
        //so make sure top 16 bits are preserved
        //during update
        this->minmax &= new_min | (0xFFFF << 16);
    }

    /**
     * @return The sensor's max
     */
    uint16_t GetMax() const
    {
        //Max is the top 16 bits of minmax
        return this->minmax.load() >> 16;
    }

    /**
     * @param new_max The sensor's new max
     */
    void SetMax(const uint16_t new_max)
    {
        //Max is the top 16 bits of minmax,
        //so make sure bottom 16 bits are
        //preserved during update
        this->minmax &= new_max | 0xFFFF;
    }

    /**
     * @return A new instance of Self with the same
     *  values as this
     */
    SensorRangeData* GetClone() const
    {
        const uint16_t value_zero = this->zero.load();
        const uint32_t value_minmax = this->minmax.load();
        const uint16_t value_min = value_minmax & 0xFFFF;
        const uint16_t value_max = value_minmax >> 16;

        return new SensorRangeData(value_zero, value_min, value_max);
    }

    /**
     * Load data into reference params
     * @param value_zero The sensor's zero
     * @param value_min The sensor's min
     * @param value_max The sensor's max
     */
    void GetValues(uint16_t& value_zero, uint16_t& value_min, uint16_t& value_max) const
    {
        value_zero = this->zero.load();
        const uint32_t value_minmax = this->minmax.load();
        value_min = value_minmax & 0xFFFF;
        value_max = value_minmax >> 16;
    }

    /**
     * Load range data into reference params
     * @param value_min The sensor's min
     * @param value_max The sensor's max
     */
    void GetRange(uint16_t& value_min, uint16_t& value_max) const
    {
        const uint32_t value_minmax = this->minmax.load();
        value_min = value_minmax & 0xFFFF;
        value_max = value_minmax >> 16;
    }

    /**
     * Update values
     * @param new_zero The sensor's new zero
     * @param new_min The sensor's new max
     * @param new_max The sensor's new min
     */
    void Update(const uint16_t new_zero, const uint16_t new_min, const uint16_t new_max)
    {
        this->zero.store(new_zero);
        this->minmax.store(new_min | (new_max << 16));
    }

    /**
     * Update values from another instance
     * @param other The instance to update from
     */
    void Update(const SensorRangeData& other)
    {
        uint16_t value_zero, value_min, value_max;
        other.GetValues(value_zero, value_min, value_max);

        Update(value_zero, value_min, value_max);
    }

    /**
     * Update range values
     * @param new_min The sensor's new max
     * @param new_max The sensor's new min
     */
    void UpdateRange(const uint16_t new_min, const uint16_t new_max)
    {
        this->minmax.store(new_min | (new_max << 16));
    }

    /**
     * Update range values from another instance of Self
     * @param other The instance to update from
     */
    void UpdateRange(const SensorRangeData& other)
    {
        uint16_t value_min, value_max;
        other.GetRange(value_min, value_max);

        UpdateRange(value_min, value_max);
    }

    /**
     * Update the current value of zero if the new zero
     * is greater than it.
     * @param new_zero The sensor's new min (if the condition is satisfied)
     * @todo Can this livelock?
     */
    void UpdateZeroIfGreater(const uint16_t new_zero)
    {
        //Update the current value for zero with the new value,
        //retrying if the current value changed during the processed
        while (true)
        {
            auto current_zero = this->zero.load();

            if (new_zero > current_zero)
            {
                if (this->zero.compare_exchange_strong(current_zero, new_zero))
                {
                    break;
                }
            }
            else
            {
                break;
            }
        }
    }

    /**
     * Update the current value for min if the new value is less, and update
     * the current value for max if the new value is greater.
     * @param new_value The sensor's new min/max (if this condition is satisfied)
     * @todo Can this livelock?
     */
    void UpdateRangeIfMinMax(const uint16_t new_value)
    {
        // //Only consider if exceeds zero
        // const auto current_zero = this->zero.load();
        //
        // if (new_value <= current_zero)
        //     return;

        //Update the current values for minmax with the new value,
        //retrying if the current value changed during the processed
        while (true)
        {
            auto current_minmax = this->minmax.load();
            const auto current_min = current_minmax & 0xFFFF;
            const auto current_max = current_minmax >> 16;

            bool changed = false;
            auto new_min = current_min;
            auto new_max = current_max;

            if (new_value < current_min)
            {
                new_min = new_value;
                changed = true;
            }

            if (new_value > current_max)
            {
                new_max = new_value;
                changed = true;
            }

            if (changed)
            {
                const auto new_minmax = new_min | (new_max << 16);

                if (this->minmax.compare_exchange_strong(current_minmax, new_minmax))
                {
                    break;
                }
            }
            else
            {
                break;
            }
        }
    }

    /**
     * @return A string describing the contained data.
     */
    std::string ToString() const
    {
        uint16_t value_zero, value_min, value_max;
        this->GetValues(value_zero, value_min, value_max);
        const uint32_t value_minmax = this->minmax.load();

        auto text = std::string(254, '\0');
        std::sprintf(&text[0], "SensorRangeData|z: %d, r: %d - %d (%d)", value_zero, value_min, value_max, value_minmax);

        return text;
    }
};

#endif //AUDIO_CONTROLLER_TOF_SENSOR_CALIBRATION_DATA_H
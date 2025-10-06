//
// Created by bendstein on 10/5/2025.
//

#ifndef AUDIO_CONTROLLER_IR_TOF_SENSOR_DRIVER_H
#define AUDIO_CONTROLLER_IR_TOF_SENSOR_DRIVER_H
#include <Arduino.h>
#include <atomic>
#include "common.h"

class TOFSensor
{
    /**
     * Round sensor values to prev multiple of this
     */
    constexpr static int ROUNDING = 16;

    /**
     * GPIO PIN
     */
    uint8_t pin;

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

    TOFSensor(const uint8_t pin, const uint16_t zero, const uint32_t minmax) : pin(pin), zero(zero), minmax(minmax) { }

public:
    TOFSensor(const uint8_t pin, const uint16_t zero, const uint16_t min, const uint16_t max) : TOFSensor(pin, zero, min | (max << 16)) { }

    TOFSensor(const TOFSensor& other) : TOFSensor(other.pin, other.zero, other.minmax) { }

    explicit TOFSensor(const uint8_t pin) : TOFSensor(pin, 0, 0, 0) { }

    TOFSensor() : TOFSensor(PIN_INVALID) {}

    /**
     * @return The current value of the attached pin
     */
    uint16_t GetValue() const
    {
        //Return 0 if invalid pin
        if (pin == PIN_INVALID)
            return 0;

        //Read current value
        const auto sensor_value = analogRead(pin);

        //Round sensor value
        return ROUNDING * static_cast<uint16_t>(std::floor(sensor_value / (ROUNDING * 1.)));
    }

    /**
     * @return The current value of the attached pin, or 0 if <= zero
     */
    uint16_t GetZeroedValue() const
    {
        const auto value = GetValue();

        if (value <= zero.load())
            return 0;

        return value;
    }

    /**
     * @return The current associated pin
     */
    uint8_t GetPin() const
    {
        return pin;
    }

    /**
     * Set the associated pin
     */
    void SetPin(const uint8_t new_pin)
    {
        pin = new_pin;
    }

    /**
     * Assign the maximum value that the sensor outputs
     * when not being interacted with
     */
    void SetZero(const uint16_t new_zero)
    {
        zero.store(new_zero);
    }

    /**
     * Assign the range of valid sensor values
     */
    void SetRange(const uint16_t new_min, const uint16_t new_max)
    {
        minmax.store(new_min | (new_max << 16));
    }

    /**
     * Get the maximum value that the sensor outputs
     * when not being interacted with
     */
    uint16_t GetZero() const
    {
        return zero.load();
    }

    /**
     * Get the range of valid values that can be outputted by the sensor,
     * and store them in the given reference parameters
     * @param out_min Reference param that min value will be put in
     * @param out_max Reference param that max value will be put in
     */
    void OutputRange(uint16_t& out_min, uint16_t& out_max) const
    {
        const auto range = minmax.load();
        out_min = range & 0xFFFF;
        out_max = range >> 16;
    }

    /**
     * Return current value of the attached pin, and output range
     * of valid values
     * @param out_min Reference param that min value will be put in
     * @param out_max Reference param that max value will be put in
     * @return The current value of the attached pin, or 0 if <= zero
     */
    uint16_t GetZeroedValueAndRange(uint16_t& out_min, uint16_t& out_max) const
    {
        OutputRange(out_min, out_max);
        return GetZeroedValue();
    }

    /**
     * Copy values from other instance of self
     */
    void CopyValuesFrom(const TOFSensor& other)
    {
        zero.store(other.zero.load());
        minmax.store(other.minmax.load());
    }

    /**
     * Copy zero from other instance of self
     */
    void CopyZeroFrom(const TOFSensor& other)
    {
        zero.store(other.zero.load());
    }

    /**
     * Copy range from other instance of self
     */
    void CopyRangeFrom(const TOFSensor& other)
    {
        minmax.store(other.minmax.load());
    }

    /**
     * Update the current value of zero if the new zero
     * is greater than it.
     * @todo Can this livelock?
     */
    void UpdateZeroIfGreater(const uint16_t new_zero)
    {
        //Update the current value for zero with the new value,
        //retrying if the current value changed during the processed
        while (true)
        {
            auto current_zero = zero.load();

            if (new_zero > current_zero)
            {
                if (zero.compare_exchange_strong(current_zero, new_zero))
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
     * @todo Can this livelock?
     */
    void UpdateRangeIfMinMax(const uint16_t new_value)
    {
        //Update the current values for minmax with the new value,
        //retrying if the current value changed during the processed
        while (true)
        {
            auto current_minmax = minmax.load();
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
};

class TOFSensorDriver
{
    static constexpr int SENSORS_COUNT = 2;
    TOFSensor sensors[SENSORS_COUNT];

public:
    explicit TOFSensorDriver(const uint8_t pins[])
    {
        for (int i = 0; i < SENSORS_COUNT; i++)
        {
            sensors[i].SetPin(pins[i]);
            sensors[i].SetZero(0);
            sensors[i].SetRange(0, 0);
        }
    }

    /**
     * Update the zeroes on all TOF sensors
     * @param clear_current_zeros If true, initially reset the zero on all sensors
     */
    void UpdateZeros(const bool clear_current_zeros = false)
    {
        for (auto &sensor : sensors)
        {
            //If flag is set, reset sensor zero to 0
            if (clear_current_zeros)
                sensor.SetZero(0);

            //Read value from sensor and apply to zero
            sensor.UpdateZeroIfGreater(sensor.GetValue());
        }
    }

    /**
     *
     * @param clear_current_range If true, initially reset the range on all sensors
     */
    void UpdateRange(const bool clear_current_range = false)
    {
        for (auto &sensor : sensors)
        {
            //If flag is set, reset sensor range to 0
            if (clear_current_range)
                sensor.SetRange(std::numeric_limits<uint16_t>::max(), 0);

            //Read value from sensor and apply to range
            sensor.UpdateRangeIfMinMax(sensor.GetValue());
        }
    }

    /**
     * @param i Sensor index
     * @return The value of the ith sensor
     */
    uint16_t GetSensorValue(const int i) const
    {
        if (i < 0 || i >= SENSORS_COUNT)
            throw std::out_of_range("Invalid sensor index.");

        return sensors[i].GetZeroedValue();
    }

    /**
     * @param i Sensor index
     * @param out_min Reference param that min value of the ith sensor will be put in
     * @param out_max Reference param that max value of the ith sensor will be put in
     * @return The value of the ith sensor
     */
    uint16_t GetSensorValue(const int i, uint16_t& out_min, uint16_t& out_max) const
    {
        if (i < 0 || i >= SENSORS_COUNT)
            throw std::out_of_range("Invalid sensor index.");

        return sensors[i].GetZeroedValueAndRange(out_min, out_max);
    }

    /**
     * @return The number of sensors
     */
    static int GetSensorCount()
    {
        return SENSORS_COUNT;
    }
};

#endif //AUDIO_CONTROLLER_IR_TOF_SENSOR_DRIVER_H
//
// Created by bendstein on 10/5/2025.
//

#ifndef AUDIO_CONTROLLER_IR_TOF_SENSOR_DRIVER_H
#define AUDIO_CONTROLLER_IR_TOF_SENSOR_DRIVER_H
#include <Arduino.h>
#include <atomic>
#include <iomanip>
#include <sstream>

#include "common.h"

class TOFSensor
{
    /**
     * Round sensor values to prev multiple of this
     */
    constexpr static int ROUNDING = 16;

    constexpr static uint16_t ZERO_DFT = 1000;

    constexpr static uint16_t MIN_DFT = 1150;

    constexpr static uint16_t MAX_DFT = INPUT_MAX_VALUE;

    /**
     * GPIO PIN
     */
    uint8_t pin = PIN_INVALID;

    /**
     * The max value that the sensor produces when not being interacted
     * with.
     */
    std::atomic_uint16_t zero { ZERO_DFT };

    /**
     * The range of values that the sensor can produce when it is
     * being interacted with. Each is a 16-bit, unsigned integer,
     * with the max as the most-significant 16 bits and the min
     * as the least-significant 16 bits.
     */
    std::atomic_uint32_t minmax { MIN_DFT | (MAX_DFT << 16) };

    TOFSensor(const uint8_t pin, const uint16_t zero, const uint32_t minmax) : pin(pin), zero(zero), minmax(minmax) { }

public:
    TOFSensor(const uint8_t pin, const uint16_t zero, const uint16_t min, const uint16_t max) : TOFSensor(pin, zero, min | (max << 16)) { }

    TOFSensor(const TOFSensor& other) : TOFSensor(other.pin, other.zero, other.minmax) { }

    explicit TOFSensor(const uint8_t pin) : TOFSensor(pin, ZERO_DFT, MIN_DFT, MAX_DFT) { }

    TOFSensor() : TOFSensor(PIN_INVALID) {}

    /**
     * @param consider_zero If true, reduce value to account for zero
     * @return The current value of the attached pin
     */
    uint16_t GetValue(const bool consider_zero = false) const
    {
        //Return 0 if invalid pin
        if (pin == PIN_INVALID)
            return 0;

        //Read current value
        const auto sensor_value = analogRead(pin);

        //Round sensor value
        auto rounded = ROUNDING * static_cast<uint16_t>(std::floor(sensor_value / (ROUNDING * 1.)));

        //Subtract zero
        if (consider_zero)
        {
            const auto current_zero = zero.load();

            if (rounded > current_zero)
                rounded -= current_zero;
            else
                rounded = 0;
        }

        return rounded;
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
     * @param consider_zero If true, reduce output values to account for zero
     */
    void OutputRange(uint16_t& out_min, uint16_t& out_max, const bool consider_zero = false) const
    {
        const auto range = minmax.load();
        out_min = range & 0xFFFF;
        out_max = range >> 16;

        if (consider_zero)
        {
            const auto current_zero = zero.load();

            if (out_min > current_zero)
                out_min -= current_zero;
            else
                out_min = 0;

            if (out_max > current_zero)
                out_max -= current_zero;
            else
                out_max = 0;
        }
    }

    /**
     * Return the current value of the pin. Get the range of valid values that can be outputted by the sensor,
     * and store them in the given reference parameters
     * @param out_min Reference param that min value will be put in
     * @param out_max Reference param that max value will be put in
     * @param consider_zero If true, reduce values to account for zero
     * @return The current value of the pin
     */
    uint16_t GetValueAndRange(uint16_t& out_min, uint16_t& out_max, const bool consider_zero = false) const
    {
        OutputRange(out_min, out_max, consider_zero);
        return GetValue(consider_zero);
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
                if (zero.compare_exchange_weak(current_zero, new_zero))
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
        //Clamp bottom to zero
        const auto current_zero = zero.load();
        const auto new_value_adj = new_value < current_zero
            ? current_zero
            : new_value;

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

            if (new_value_adj < current_min)
            {
                new_min = new_value_adj;
                changed = true;
            }

            if (new_value_adj > current_max)
            {
                new_max = new_value_adj;
                changed = true;
            }

            if (changed)
            {
                const auto new_minmax = new_min | (new_max << 16);

                if (this->minmax.compare_exchange_weak(current_minmax, new_minmax))
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

    std::string ToString() const
    {
        std::stringstream stream {};
        WriteToStringStream(stream);
        return stream.str();
    }

    void WriteToStringStream(std::stringstream& stream) const
    {
        uint16_t value_min, value_max;
        OutputRange(value_min, value_max);

        stream << "Sensor| Pin: " << static_cast<uint16_t>(pin)
            << "; Zero: " << zero.load()
            << "; Min: " << value_min
            << "; Max: " << value_max
            << "; MinMax: " << minmax.load();
            ;
    }
};

class TOFSensorDriver
{
public:
    static constexpr int SENSORS_COUNT = 2;

private:
    TOFSensor sensors[SENSORS_COUNT];

public:
    explicit TOFSensorDriver(const uint8_t pins[SENSORS_COUNT])
    {
        for (int i = 0; i < SENSORS_COUNT; i++)
        {
            sensors[i].SetPin(pins[i]);
            sensors[i].SetZero(0);
            sensors[i].SetRange(0, INPUT_MAX_VALUE);
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
            const auto value = sensor.GetValue();

            //If flag is set, reset sensor zero to 0
            if (clear_current_zeros)
                sensor.SetZero(0);

            //Read value from sensor and apply to zero
            sensor.UpdateZeroIfGreater(value);
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
            const auto value = sensor.GetValue();

            //If flag is set, reset sensor range to 0
            if (clear_current_range)
                sensor.SetRange(std::numeric_limits<uint16_t>::max(), 0);

            //Read value from sensor and apply to range
            sensor.UpdateRangeIfMinMax(value);
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

        return sensors[i].GetValue(true);
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

        return sensors[i].GetValueAndRange(out_min, out_max, true);
    }

    std::string ToString(const bool write_value = false) const
    {
        std::stringstream stream {};

        WriteToStringStream(stream, write_value);

        return stream.str();
    }

    void WriteToStringStream(std::stringstream &stream, const bool write_value = false) const
    {
        stream << "Sensors:" << std::endl;

        for (auto &sensor : sensors)
        {
            stream << "  - ";
            sensor.WriteToStringStream(stream);

            if (write_value)
            {
                stream << " [" << sensor.GetValue(true) << "]";
            }

            stream << std::endl;
        }
    }
};

#endif //AUDIO_CONTROLLER_IR_TOF_SENSOR_DRIVER_H
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
    constexpr static uint16_t ROUNDING_CM = 2;

    uint8_t pin = PIN_INVALID;
    std::atomic_uint32_t minmax { CombineU16(ZERO_DFT_MV, MAX_DFT_CM) };

    TOFSensor(const uint8_t pin, const uint32_t minmax) : pin(pin), minmax(minmax) {}
public:
    constexpr static uint16_t ZERO_DFT_MV = 500;
    constexpr static uint16_t ABS_MIN_DIST_V = 1;
    constexpr static uint16_t MAX_DFT_CM = 60;

    TOFSensor(const uint8_t pin, const uint16_t zero_mv, const uint16_t max_cm) : TOFSensor(pin, CombineU16(zero_mv, max_cm)) {}
    explicit TOFSensor(const uint8_t pin) : TOFSensor(pin, ZERO_DFT_MV, MAX_DFT_CM) {}
    TOFSensor() : TOFSensor(PIN_INVALID) {}
    TOFSensor(const TOFSensor &other) : TOFSensor(other.pin, other.minmax) {}

    uint8_t GetPin() const { return pin; }
    void SetPin(const uint8_t new_pin) { pin = new_pin; }

    uint16_t GetZero() const { return U32GetComponentU16(minmax.load(), false); }
    void SetZero(const uint16_t new_zero)
    {
        //Update zero component of minmax, retrying if changed elsewhere during attempt
        while (true)
        {
            auto current_minmax = minmax.load();
            const auto new_minmax = U32UpdateU16(current_minmax, new_zero, false);

            if (minmax.compare_exchange_weak(current_minmax, new_minmax))
                break;
        }
    }

    uint16_t GetMax() const { return U32GetComponentU16(minmax.load(), true); }
    void SetMax(const uint16_t new_max_cm)
    {
        //Update max component of minmax, retrying if changed elsewhere during attempt
        while (true)
        {
            auto current_minmax = minmax.load();
            const auto new_minmax = U32UpdateU16(current_minmax, new_max_cm, true);

            if (minmax.compare_exchange_weak(current_minmax, new_minmax))
                break;
        }
    }

    void GetBounds(uint16_t &zero_mv, uint16_t &max_cm) const
    {
        SplitU32(minmax.load(), zero_mv, max_cm);
    }
    
    uint16_t GetCurrentValueMillivolts() const
    {
        //Return 0 if invalid pin
        if (pin == PIN_INVALID)
            return 0;

        //Read current value in millivolts
        //analogReadMilliVolts returns an uint32, but it should never
        //get anywhere near the max uint16, so will cast to uint16
        const auto sensor_value = static_cast<uint16_t>(std::min(static_cast<uint32_t>(std::numeric_limits<uint16_t>::max()), analogReadMilliVolts(pin)));

        return sensor_value;
    }

    uint16_t GetCurrentValueMillivolts(uint16_t &zero_mv, uint16_t &max_cm) const
    {
        GetBounds(zero_mv, max_cm);
        return GetCurrentValueMillivolts();
    }

    uint16_t GetCurrentDistance() const
    {
        const auto current_value_millivolts = GetCurrentValueMillivolts();
        return GetDistance(current_value_millivolts, GetZero());
    }

    uint16_t GetCurrentDistance(uint16_t &zero_mv, uint16_t &max_cm) const
    {
        GetBounds(zero_mv, max_cm);
        return GetCurrentDistance();
    }
    uint16_t GetCurrentDistance(uint16_t &zero_mv, uint16_t &max_cm, uint16_t &value_mv) const
    {
        GetBounds(zero_mv, max_cm);
        value_mv = GetCurrentValueMillivolts();
        return GetDistance(value_mv, zero_mv);
    }

    void UpdateZeroIfGreater(const uint16_t new_zero_mv)
    {
        //If new zero is greater than current zero, update zero component of minmax,
        //retrying if changed elsewhere during attempt
        while (true)
        {
            auto current_minmax = minmax.load();
            const auto current_zero = U32GetComponentU16(current_minmax, false);

            if (new_zero_mv > current_zero)
            {
                const auto new_minmax = U32UpdateU16(current_minmax, new_zero_mv, false);
                
                if (minmax.compare_exchange_weak(current_minmax, new_minmax))
                    break;   
            }
            else
            {
                break;
            }
        }
    }
    
    void UpdateMaxIfGreater(const uint16_t new_max_cm)
    {
        //If new max is greater than current max, update max component of minmax,
        //retrying if changed elsewhere during attempt
        while (true)
        {
            auto current_minmax = minmax.load();
            const auto current_max = U32GetComponentU16(current_minmax, true);

            if (new_max_cm > current_max)
            {
                const auto new_minmax = U32UpdateU16(current_minmax, new_max_cm, true);
                
                if (minmax.compare_exchange_weak(current_minmax, new_minmax))
                    break;   
            }
            else
            {
                break;
            }
        }
    }

    std::string ToString() const
    {
        std::stringstream stream{};
        WriteToStringStream(stream);
        return stream.str();
    }

    void WriteToStringStream(std::stringstream& stream) const
    {
        uint16_t current_zero_mv = 0, current_max_cm = 0;
        const auto current_minmax = minmax.load();
        SplitU32(current_minmax, current_zero_mv, current_max_cm);

        stream << "Sensor | Pin: " << static_cast<uint16_t>(pin)
            << "; Zero: " << current_zero_mv << "mV"
            << "; Max: " << current_max_cm << "cm"
            << "; MinMax: " << current_minmax;
        ;
    }

    static uint16_t GetDistance(const uint16_t millivolts, const uint16_t zero_millivolts)
    {
        const double volts = millivolts / 1000.;
        const double zero_volts = zero_millivolts / 1000.;

        //From datasheet - a voltage below about 0.4 is ~0. Actual value
        //is value in variable zero_volts
        constexpr double EXPECTED_MIN_VOLTAGE = 0.4;

        if (volts <= zero_volts)
            return 0;

        //Adjust voltage by zero error
        const auto min_voltage_err = zero_volts - EXPECTED_MIN_VOLTAGE;
        const double volts_adj = volts + min_voltage_err;

        /*
         * For the purposes of distance, don't consider anything below voltage threshold
         * Per the datasheet, all voltages below the max correspond to two distances
         * (e.g. 2V ~= 30cm and 9cm). Even though it doesn't seem to cause a problem
         * while sensing an obstruction (or when no obstruction), it causes false
         * readings when the capacitor drains. This check helps mitigate that.
         *
         * The actual distance of 1V is ~63cm, so anything with a distance of
         * > ~63cm won't be picked up.
         */
        if (volts_adj < ABS_MIN_DIST_V)
            return 0;

        //Lines of best fit based on data in datasheet.
        //More accurate if use a different function for
        //each side of the cutoff
        constexpr double CUTOFF = 0.905; //0.905V ~= 70cm
        const double distance_cm = volts_adj < CUTOFF
            ? 249 * std::pow(volts_adj, 2) - 500 * volts_adj + 320
            : 11.7 * std::pow(volts_adj, 2) - 70.2 * volts_adj + 122;

        //Round value
        const double rounded = ROUNDING_CM * std::round(distance_cm / ROUNDING_CM);

        //For any value that the sensor can actually produce (~3V max), distance
        //won't be more than 150cm. Clamp for safety, but not actually a concern.
        return static_cast<uint16_t>(
            std::max(
                0.,
                std::min(
                    1. * std::numeric_limits<uint16_t>::max(),
                    rounded
                )
            )
        );
    }
private:
    static uint32_t CombineU16(const uint16_t a, const uint16_t b) { return a | (b << 16); }

    static void SplitU32(const uint32_t value, uint16_t &a, uint16_t &b)
    {
        a = value & 0xFFFF;
        b = value >> 16;
    }

    static uint16_t U32GetComponentU16(const uint32_t value, const bool top)
    {
        if (top)
            return value >> 16;
        return value & 0xFFFF;
    }

    static uint32_t U32UpdateU16(const uint32_t value, const uint16_t new_part, const bool top)
    {
        if (top)
            return (value & 0xFFFF) | (new_part << 16);
        return (value & 0xFFFF0000) | new_part;
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
            sensors[i].SetZero(TOFSensor::ZERO_DFT_MV);
            sensors[i].SetMax(TOFSensor::MAX_DFT_CM);
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
            const auto value = sensor.GetCurrentValueMillivolts();
            sensor.UpdateZeroIfGreater(value);
        }
    }

    /**
     *
     * @param clear_current_max If true, initially reset the max distance on all sensors
     */
    void UpdateMax(const bool clear_current_max = false)
    {
        for (auto &sensor : sensors)
        {
            //If flag is set, reset sensor range to 0
            if (clear_current_max)
                sensor.SetMax(0);

            //Read value from sensor and apply to range
            const auto distance = sensor.GetCurrentDistance();
            sensor.UpdateMaxIfGreater(distance);
        }
    }

    /**
     * @param i Sensor index
     * @return The value of the ith sensor
     */
    uint16_t GetSensorValueMillivolts(const int i) const
    {
        if (i < 0 || i >= SENSORS_COUNT)
            throw std::out_of_range("Invalid sensor index.");

        return sensors[i].GetCurrentValueMillivolts();
    }

    /**
     * @param i Sensor index
     * @return The value of the ith sensor, converted to cm
     */
    uint16_t GetSensorDistance(const int i) const
    {
        if (i < 0 || i >= SENSORS_COUNT)
            throw std::out_of_range("Invalid sensor index.");

        return sensors[i].GetCurrentDistance();
    }

    /**
     * @param i Sensor index
     * @param out_zero_mv Reference param that zero of the ith sensor will be put in
     * @param out_max_cm Reference param that max value of the ith sensor will be put in
     * @return The value of the ith sensor
     */
    uint16_t GetSensorValueMillivolts(const int i, uint16_t& out_zero_mv, uint16_t& out_max_cm) const
    {
        if (i < 0 || i >= SENSORS_COUNT)
            throw std::out_of_range("Invalid sensor index.");

        return sensors[i].GetCurrentValueMillivolts(out_zero_mv, out_max_cm);
    }

    /**
     * @param i Sensor index
     * @param out_zero_mv Reference param that zero of the ith sensor will be put in
     * @param out_max_cm Reference param that max value of the ith sensor will be put in
     * @return The distance read by the ith sensor
     */
    uint16_t GetSensorDistance(const int i, uint16_t& out_zero_mv, uint16_t& out_max_cm) const
    {
        if (i < 0 || i >= SENSORS_COUNT)
            throw std::out_of_range("Invalid sensor index.");

        return sensors[i].GetCurrentDistance(out_zero_mv, out_max_cm);
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
                uint16_t zero_mv = 0, max_cm = 0, value_mv = 0;
                const auto distance = sensor.GetCurrentDistance(zero_mv, max_cm, value_mv);
                const auto ratio = max_cm == 0
                    ? 0
                    : static_cast<uint16_t>(std::round((100. * distance) / max_cm));

                stream << " [" << value_mv << "mV / " << distance << "cm - " << ratio << "%]";
            }

            stream << std::endl;
        }
    }
};

#endif //AUDIO_CONTROLLER_IR_TOF_SENSOR_DRIVER_H
//
// Created by bendstein on 11/2/2025.
//

#ifndef AUDIO_CONTROLLER_TOF_DRIVER_H
#define AUDIO_CONTROLLER_TOF_DRIVER_H
#include <Arduino.h>
#include <iomanip>
#include <sstream>

#include "common.h"

#define SENSOR_TYPE__GP2Y0A02YK
// #define SENSOR_TYPE__GP2Y0E02A

class SensorArrayDriver
{
public:
    static constexpr uint8_t SELECT_PIN_COUNT = 3;
    static constexpr uint8_t MAX_SENSOR_COUNT = SELECT_PIN_COUNT == 0? 0 : (1 << SELECT_PIN_COUNT);
    static constexpr uint32_t SELECT_DELAY_MICRO = 50;

    static const uint16_t SENSOR_MIN_MV;
    static const double SENSOR_MIN_V;
    static const uint16_t SENSOR_MAX_MV;
    static const uint16_t SENSOR_MIN_DISTANCE;
    static const uint16_t SENSOR_MAX_DISTANCE;

    static uint16_t SensorValueToDistance__Specific(uint16_t value_mv);

private:
    const uint8_t data_pin = PIN_INVALID;
    uint8_t select_pins[SELECT_PIN_COUNT] = {};
    uint8_t effective_select_pin_count = SELECT_PIN_COUNT;
    uint8_t effective_max_sensor_count = MAX_SENSOR_COUNT;

public:
    SensorArrayDriver(const uint8_t data_pin, const uint8_t select_pins[SELECT_PIN_COUNT]) : data_pin(data_pin)
    {
        for (auto i = 0; i < SELECT_PIN_COUNT; i++)
        {
            this->select_pins[i] = select_pins[i];

            //Don't consider for invalid pins
            if (select_pins[i] == PIN_INVALID)
            {
                effective_select_pin_count = i;
                effective_max_sensor_count = 1 << effective_select_pin_count;
                break;
            }
        }

        effective_select_pin_count = 0;
        effective_max_sensor_count = 1;
    }

    uint8_t EffectiveSelectPinCount() const
    {
        return effective_select_pin_count;
    }

    uint8_t EffectiveMaxSensorCount() const
    {
        return effective_max_sensor_count;
    }

    uint16_t ReadSensor(const uint8_t sensor) const
    {
        //Write to select pins
        for (auto b = 0; b < effective_select_pin_count; b++)
        {
            digitalWrite(select_pins[b], sensor & (1 << b));
        }

        //Delay to give multiplexer time to update
        delayMicroseconds(SELECT_DELAY_MICRO);

        //Read new sensor value
        const auto sensor_value = analogReadMilliVolts(data_pin);

        //Sensor value should never get anywhere near the max ushort
        assert(sensor_value <= std::numeric_limits<uint16_t>::max());

        return sensor_value;
    }

    uint16_t ReadSensor(const uint8_t sensor, uint16_t& distance) const
    {
        const auto value = ReadSensor(sensor);
        distance = SensorValueToDistance(value);

        return value;
    }

    uint16_t ReadSensorDistance(const uint8_t sensor) const
    {
        return SensorValueToDistance(ReadSensor(sensor));
    }

    std::string ToString() const
    {
        std::stringstream stream {};
        WriteToStringStream(stream);
        return stream.str();
    }

    void WriteToStringStream(std::stringstream& stream) const
    {
        stream << "SensorArray | "
            << "Data Pin: " << static_cast<uint32_t>(data_pin) << "; "
            << "Select Pins (" << static_cast<uint32_t>(effective_select_pin_count) << "): ";

        for (auto i = 0; i < effective_select_pin_count; i++)
        {
            stream << static_cast<uint32_t>(select_pins[i]);
            if (i < effective_select_pin_count - 1)
                stream << ", ";
        }

        stream << "; Sensor count: " << static_cast<uint32_t>(effective_max_sensor_count) << std::endl
            << std::fixed << std::showpoint << std::setprecision(2);

        for (auto i = 0; i < effective_max_sensor_count; i++)
        {
            uint16_t distance;
            const auto value = ReadSensor(i, distance);

            stream << "  - " << i << ": " << value << "mV (" << distance << "cm)" << std::endl;
        }
    }
    
    static uint16_t SensorValueToDistance(const uint16_t value_mv)
    {
        return SensorValueToDistance__Specific(value_mv);
    }
};

#ifdef SENSOR_TYPE__GP2Y0A02YK
    constexpr uint16_t SensorArrayDriver::SENSOR_MIN_MV = 400; //~400mV zero, per datasheet
    constexpr uint16_t SensorArrayDriver::SENSOR_MAX_MV = 2750; //~2750mV max output, per datasheet
    constexpr double SensorArrayDriver::SENSOR_MIN_V = SENSOR_MIN_MV / 1000.;

    /**
     * Distance corresponding to given GP2Y0A02YK sensor voltage
     */
    inline uint16_t SensorArrayDriver::SensorValueToDistance__Specific(const uint16_t value_mv)
    {
        //Value may exceed maximum due to approximation, but clamp to max to prevent
        //unexpected values
        const double volts = (value_mv >= SENSOR_MAX_MV? SENSOR_MAX_MV : value_mv) / 1000.;

        if (volts <= SENSOR_MIN_V)
            return 0;

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
        static constexpr double ABS_MIN_DIST_V = 1.;

        if (volts < ABS_MIN_DIST_V)
            return 0;

        //Lines of best fit based on data in datasheet.
        //More accurate if use a different function for
        //each side of the cutoff
        constexpr double CUTOFF = 0.905; //0.905V ~= 70cm
        const double distance_cm = volts < CUTOFF
            ? 249 * std::pow(volts, 2) - 500 * volts + 320
            : 11.7 * std::pow(volts, 2) - 70.2 * volts + 122;

        //Round value
        static constexpr uint16_t ROUNDING_CM = 2;
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
#else
#ifdef SENSOR_TYPE__GP2Y0E02A
    constexpr uint16_t SensorArrayDriver::SENSOR_MIN_MV = 0;
    constexpr uint16_t SensorArrayDriver::SENSOR_MAX_MV = 1;
    constexpr double SensorArrayDriver::SENSOR_MIN_V = SENSOR_MIN_MV / 1000.;

    /**
     * Distance corresponding to given GP2Y0E02A sensor voltage
     */
    inline uint16_t SensorArrayDriver::SensorValueToDistance__Specific(const uint16_t value_mv)
    {
        return 0;
    }
#else
    constexpr uint16_t SENSOR_MIN_MV = 0;
    constexpr uint16_t SENSOR_MAX_MV = 1;
    constexpr double SENSOR_MIN_V = SENSOR_MIN_MV / 1000.;

    /**
     * Distance corresponding to given sensor voltage, if no sensor
     * is defined
     */
    inline uint16_t SensorArrayDriver::SensorValueToDistance__Specific(const uint16_t value_mv)
    {
        return 0;
    }
#endif
#endif

const uint16_t SensorArrayDriver::SENSOR_MIN_DISTANCE = SensorArrayDriver::SensorValueToDistance(SensorArrayDriver::SENSOR_MIN_MV);
const uint16_t SensorArrayDriver::SENSOR_MAX_DISTANCE = SensorArrayDriver::SensorValueToDistance(SensorArrayDriver::SENSOR_MAX_MV);

#endif //AUDIO_CONTROLLER_TOF_DRIVER_H
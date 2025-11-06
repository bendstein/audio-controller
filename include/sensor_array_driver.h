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
    static constexpr uint32_t SELECT_DELAY_MICRO = 100;

    static const uint16_t SENSOR_MIN_MV;
    static const double SENSOR_MIN_V;
    static const uint16_t SENSOR_MAX_MV;
    static const uint16_t SENSOR_MIN_DISTANCE;
    static const uint16_t SENSOR_MAX_DISTANCE;
    static const uint16_t SENSOR_DETACHED_THRESHOLD_MV;

    static uint16_t SensorValueToDistance__Specific(uint16_t value_mv);

private:
    const uint8_t data_pin = PIN_INVALID;
    uint8_t select_pins[SELECT_PIN_COUNT] = {};
    uint16_t active_sensors = 0;
    uint8_t effective_select_pin_count = SELECT_PIN_COUNT;
    uint8_t effective_max_sensor_count = MAX_SENSOR_COUNT;

public:
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
        //Skip inactive sensor
        if (!IsSensorActive(sensor))
            return 0;

        //Write to select pins
        for (auto b = 0; b < effective_select_pin_count; b++)
        {
            const auto shift = 1 << b;
            digitalWrite(select_pins[b], (sensor & shift) == shift? HIGH : LOW);
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

    bool IsSensorActive(const uint8_t sensor) const
    {
        const auto shift = 1 << sensor;
        return (active_sensors & shift) == shift;
    }

    void RefreshActiveSensors()
    {
        //Reinitialize the number of valid select pins and active sensors
        //so that we can properly read the values for all potentially connected sensors.
        //Clear any select pins
        active_sensors = (1 << MAX_SENSOR_COUNT) - 1;

        uint8_t select_pin_count = SELECT_PIN_COUNT;

        for (auto i = 0; i < SELECT_PIN_COUNT; i++)
        {
            if (select_pins[i] == PIN_INVALID)
            {
                //Only consider up to first invalid pin
                //(but continue so any following pins can
                //be cleared)
                if (select_pin_count != SELECT_PIN_COUNT)
                    select_pin_count = i;
                continue;
            }

            digitalWrite(select_pins[i], LOW);
        }

        effective_select_pin_count = select_pin_count;

        //Check all potentially attached sensors.
        //If their value is below a set threshold,
        //consider the position to be empty
        uint16_t sensors = 0;
        for (auto i = 0; i < MAX_SENSOR_COUNT; i++)
        {
            const auto sensor_value = ReadSensor(i);

            if (sensor_value > SENSOR_DETACHED_THRESHOLD_MV)
            {
                sensors |= (1 << i);
            }
        }

        //Clear select pins to prevent unused ones from staying on
        for (auto i = 0; i < SELECT_PIN_COUNT; i++)
        {
            if (select_pins[i] != PIN_INVALID)
            {
                digitalWrite(select_pins[i], LOW);
            }
        }

        active_sensors = sensors;
        effective_max_sensor_count = GetMaxSetBit(sensors);
        effective_select_pin_count = effective_max_sensor_count <= 1
            ? effective_max_sensor_count
            : GetMaxSetBit(static_cast<uint8_t>(effective_max_sensor_count - 1));
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

        stream << std::endl << "Attached Sensors: ";

        for (auto i = 15; i >= 0; i--)
        {
            const auto shift = 1 << i;
            stream << static_cast<uint32_t>((active_sensors & shift) == shift? 1 : 0);
        }

        stream << std::endl;

        stream << "Sensor count: " << static_cast<uint32_t>(CountSetBits(active_sensors))
            << " (Effective: " << static_cast<uint32_t>(effective_max_sensor_count) << ")"
            << std::endl << std::fixed << std::showpoint << std::setprecision(2);

        for (auto i = 0; i < effective_max_sensor_count; i++)
        {
            if (IsSensorActive(i))
            {
                uint16_t distance;
                const auto value = ReadSensor(i, distance);

                stream << "  - " << i << ": " << value << "mV (" << distance << "cm)" << std::endl;
            }
            else
            {
                stream << "  - " << i << ": <inactive>" << std::endl;
            }
        }
    }
    
    static uint16_t SensorValueToDistance(const uint16_t value_mv)
    {
        return SensorValueToDistance__Specific(value_mv);
    }

    SensorArrayDriver(const uint8_t pin_data, const uint8_t pins_select[SELECT_PIN_COUNT])
        : data_pin(pin_data), effective_select_pin_count(0), effective_max_sensor_count(0)
    {
        for (auto i = 0; i < SELECT_PIN_COUNT; i++)
        {
            select_pins[i] = pins_select[i];
        }
    }
};

#ifdef SENSOR_TYPE__GP2Y0A02YK
    constexpr uint16_t SensorArrayDriver::SENSOR_MIN_MV = 400; //~400mV zero, per datasheet
    constexpr uint16_t SensorArrayDriver::SENSOR_MAX_MV = 2750; //~2750mV max output, per datasheet
    constexpr uint16_t SensorArrayDriver::SENSOR_MIN_DISTANCE = 15; //Closest distance in cm (peak mV), per datasheet
    constexpr uint16_t SensorArrayDriver::SENSOR_MAX_DISTANCE = 150; //Furthest distance in cm (bottom mV excluding very close), per datasheet
    constexpr double SensorArrayDriver::SENSOR_MIN_V = SENSOR_MIN_MV / 1000.;
    constexpr uint16_t SensorArrayDriver::SENSOR_DETACHED_THRESHOLD_MV = 200;
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
    constexpr uint16_t SensorArrayDriver::SENSOR_DETACHED_THRESHOLD_MV = 200;

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
    constexpr uint16_t SensorArrayDriver::SENSOR_DETACHED_THRESHOLD_MV = 0;

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

#endif //AUDIO_CONTROLLER_TOF_DRIVER_H
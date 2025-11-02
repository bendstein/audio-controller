//
// Created by bendstein on 11/2/2025.
//

#ifndef AUDIO_CONTROLLER_TOF_DRIVER_H
#define AUDIO_CONTROLLER_TOF_DRIVER_H
#include <Arduino.h>
#include <iomanip>
#include <sstream>

#include "common.h"

class SensorArrayDriver
{
public:
    constexpr static uint8_t SELECT_PIN_COUNT = 3;
    constexpr static uint8_t MAX_SENSOR_COUNT = SELECT_PIN_COUNT == 0? 0 : (1 << SELECT_PIN_COUNT);
    constexpr static uint32_t SELECT_DELAY_MICRO = 50;

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
                effective_max_sensor_count = i == 0? 0 : (1 << effective_select_pin_count);
                break;
            }
        }
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

    uint16_t ReadSensor(const uint8_t sensor, double& distance) const
    {
        const auto value = ReadSensor(sensor);
        distance = SensorValueToDistance(value);

        return value;
    }

    double ReadSensorDistance(const uint8_t sensor) const
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
            << "Select Pins (" << effective_select_pin_count << "): ";

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
            double distance;
            const auto value = ReadSensor(i, distance);

            stream << "  - " << i << ": " << value << "mV (" << distance << "cm)" << std::endl;
        }
    }

    static double SensorValueToDistance(uint16_t value_mv)
    {
        return 0;
    }
};

#endif //AUDIO_CONTROLLER_TOF_DRIVER_H
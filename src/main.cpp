#include <Arduino.h>

#include "main.h"
#include "notes.h"
#include "common.h"
#include "ir_tof_sensor_driver.h"

//Whether sensors are being calibrated, and if so, what kind (zeroes/vs active range)
CalibrationState calibration_state = CalibrationState::None;

//Attached TOF sensors
constexpr uint8_t sensor_pins[] = { PIN_IN_TOF_0, PIN_IN_TOF_1 };
auto sensors = TOFSensorDriver(sensor_pins);

//Range of output frequencies to map sensors to
const FrequencyRangeData* frequency_ranges[]
{
    new FrequencyRangeData(GetMusicalNoteFrequency(MusicalNote::A, 4), GetMusicalNoteFrequency(MusicalNote::A, 4)),
    new FrequencyRangeData(GetMusicalNoteFrequency(MusicalNote::C, 4), GetMusicalNoteFrequency(MusicalNote::C, 4))
};

//The audio frequency to play based on the last-read input from sensors
std::atomic<double>* frequencies[] = {
    new std::atomic<double>(frequency_ranges[0]->GetMin()),
    new std::atomic<double>(frequency_ranges[1]->GetMin())
};

//Value (out of INPUT_MAX_VALUE) representing to volume of the output audio
auto volume = new std::atomic_uint16_t(INPUT_MAX_VALUE);

void setup() {
    Serial.begin(9600);

    try
    {
        //Setup pins
        pinMode(PIN_IN_TOF_0, PULLDOWN | INPUT);
        pinMode(PIN_IN_TOF_1, PULLDOWN | INPUT);
        pinMode(PIN_IN_ZERO, PULLDOWN | INPUT);
        pinMode(PIN_IN_CALIBRATE, PULLDOWN | INPUT);
        pinMode(PIN_IN_VOLUME, PULLDOWN | INPUT);

        pinMode(PIN_OUT_AUDIO_0, OUTPUT);
        pinMode(PIN_OUT_AUDIO_1, OUTPUT);
        pinMode(PIN_OUT_INDICATE_ZERO, OUTPUT);
        pinMode(PIN_OUT_INDICATE_CAL, OUTPUT);

        //Setup timer
        const auto timer = timerBegin(0, 50, true);
        timerAttachInterrupt(timer, isr_timer, true);
        timerAlarmWrite(timer, TIMER_INTERVAL, true);

        timerAlarmEnable(timer);
    }
    catch (std::exception& e)
    {
        Serial.println("An error occurred during setup.");
        Serial.println(e.what());
    }
}

void loop()
{
    try
    {
        /*
        //Update zeroes/calibrate sensor ranges
        const auto new_calibration_state = handle_calibration();

        //Update calibration indicators
        digitalWrite(PIN_OUT_INDICATE_ZERO, new_calibration_state == CalibrationState::Zeroing? HIGH : LOW);
        digitalWrite(PIN_OUT_INDICATE_CAL, new_calibration_state == CalibrationState::Calibrating? HIGH : LOW);

        //Is calibrating/zeroing; clear audio channel and return
        if (calibration_state != CalibrationState::None)
        {
            dacWrite(AUDIO_CHANNEL_0, 0);
            dacWrite(AUDIO_CHANNEL_1, 0);
            return;
        }
        */

        //At this point, not calibrating

        const auto t = micros();

        //Get and update current volume
        const auto new_volume = analogRead(PIN_IN_VOLUME);
        volume->store(new_volume);

        //Get ratio to max volume
        const auto volume_scale = new_volume / (INPUT_MAX_VALUE * 1.);

        //Calculate next value in wave based on current frequency values
        

        // //Calculate next value in wave based on current frequency values.
        // const double current_frequency_0 = frequency_0->load();
        // const double current_frequency_1 = frequency_1->load();
        //
        // //Calculate wave for each channel. Can put multiple frequencies in
        // //each chord, don't need to be the same
        // const double chord_0[] = { current_frequency_0, current_frequency_1 };
        // const double chord_1[] = { current_frequency_0, current_frequency_1 };
        //
        // const auto wave_value_0 = wave(t, volume_scale, chord_0, sizeof(chord_0) / sizeof(chord_0[0]));
        // const auto wave_value_1 = wave(t, volume_scale, chord_1, sizeof(chord_1) / sizeof(chord_1[0]));
        //
        // dacWrite(AUDIO_CHANNEL_0, wave_value_0);
        // dacWrite(AUDIO_CHANNEL_1, wave_value_1);
    }
    catch (std::exception& e)
    {
        Serial.println("An error occurred during main loop.");
        Serial.println(e.what());
    }
}

uint8_t wave(const uint64_t t, const double scale, const double chord[], const uint8_t chord_len)
{
    //microseconds => seconds
    const auto ts = static_cast<long double>(t) / 1000000;

    //Sum up chord components
    long double f = 0;
    uint8_t count = 0;

    for (auto i = 0; chord != nullptr && i < chord_len; i++)
    {
        const auto chord_value = chord[i];

        //Ignore non-positive frequencies.
        //0 indicates explicitly to ignore,
        //negative is invalid
        if (chord_value <= 0)
            continue;

        //Calculate sine wave for this frequency and time, scaled for volume
        const auto value = scale * sin(2 * PI * chord_value * ts);
        f += value;
        count += 1;
    }

    //Translate the waves to be centered at scale instead of at 0,
    //to prevent negative values
    if (count > 0)
    {
        f = scale + f / count;
    }

    //Map output value to a value between 0 and 255
    const auto v = static_cast<int32_t>(std::floor((DAC_MAX / 2.) * f));
    return static_cast<uint8_t>(std::max(0, std::min(DAC_MAX, v)));
}

void update_frequency()
{
    try
    {
        //Update frequencies based on sensor values, if not calibrating
        if (calibration_state == CalibrationState::None)
        {
            for (auto i = 0; i < TOFSensorDriver::GetSensorCount(); i++)
            {
                frequencies[i]->store(calculate_frequency(i, frequency_ranges[i]));
            }
        }
    }
    catch (std::exception& e)
    {
        Serial.println("An error occurred while updating frequency on timer.");
        Serial.println(e.what());
    }
}

double calculate_frequency(const int sensor, const FrequencyRangeData* frequency_range)
{
    uint16_t sensor_min, sensor_max;
    const auto sensor_value = sensors.GetSensorValue(sensor, sensor_min, sensor_max);

    //Invalid range of sensor values
    if (sensor_max <= sensor_min)
        return 0;

    //Clamp value to sensor range max
    const double sensor_value_clamped = sensor_value > sensor_max
        ? sensor_max
        : sensor_value;

    //Get ratio into range that sensor value is at
    const auto ratio = (sensor_value_clamped - sensor_min) / (sensor_max - sensor_min);

    //Map ratio to range of frequencies
    double frequency_min, frequency_max;
    frequency_range->GetValues(frequency_min, frequency_max);

    const auto calculated_frequency = ((frequency_max - frequency_min) * ratio) + frequency_min;

    return calculated_frequency;
}

CalibrationState handle_calibration()
{
    //Get current state for calibration
    const auto current_calibration_state = calibration_state;

    //Determine what state should be based on pin values
    const auto is_pin_zero = digitalRead(PIN_IN_ZERO) == HIGH;
    const auto is_pin_calibrate = digitalRead(PIN_IN_CALIBRATE) == HIGH;

    CalibrationState next_calibration_state = CalibrationState::None;

    if (is_pin_zero)
    {
        next_calibration_state = CalibrationState::Zeroing;
    }
    else if (is_pin_calibrate)
    {
        next_calibration_state = CalibrationState::Calibrating;
    }

    const auto state_change = next_calibration_state != current_calibration_state;

    //Read sensor values and update zeroes/range
    if (next_calibration_state != CalibrationState::None)
    {
        switch (next_calibration_state)
        {
            case CalibrationState::Zeroing: //Update zeroes
                sensors.UpdateZeros(state_change);
                break;
            case CalibrationState::Calibrating: //Update input ranges
                sensors.UpdateRange(state_change);
                break;
            default: ;
        }
    }

    //Update and return new calibration state
    calibration_state = next_calibration_state;
    return next_calibration_state;
}

uint16_t read_tof_pin(const uint8_t pin)
{
    const auto sensor_value = analogRead(pin);

    //Round the sensor value to the previous multiple of TOF_ROUNDING
    return TOF_ROUNDING * static_cast<uint16_t>(std::floor(sensor_value / (TOF_ROUNDING * 1.)));
}

void isr_timer()
{
    //Calculate new frequencies from sensor values on interval
    update_frequency();
}

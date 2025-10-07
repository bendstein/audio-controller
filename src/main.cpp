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

constexpr uint8_t audio_channels[] = { AUDIO_CHANNEL_0, AUDIO_CHANNEL_1 };

//Value (out of INPUT_MAX_VALUE) representing to volume of the output audio
auto volume = new std::atomic_uint16_t(INPUT_MAX_VALUE);

//Flags
volatile std::atomic_bool flag_isr_calibration_zero {};
volatile std::atomic_bool flag_isr_calibration_range {};

void setup() {
    Serial.begin(9600);
    Serial.println();

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

        //Clear flags
        flag_isr_calibration_zero.store(false);
        flag_isr_calibration_range.store(false);

        //Setup interrupts
        attachInterrupt(digitalPinToInterrupt(PIN_IN_ZERO), isr_zero, RISING);
        attachInterrupt(digitalPinToInterrupt(PIN_IN_CALIBRATE), isr_calibrate, RISING);

        //Zero
        sensors.UpdateZeros();
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
        //Handle calibration flags
        auto handle_calibration_type = CalibrationState::None;

        auto flag_isr_calibration_zero_current = flag_isr_calibration_zero.load();
        if (flag_isr_calibration_zero_current) //Toggle zeroing
        {
            flag_isr_calibration_zero.compare_exchange_strong(flag_isr_calibration_zero_current, false);
            handle_calibration_type = CalibrationState::Zeroing;
        }

        auto flag_isr_calibration_range_current = flag_isr_calibration_range.load();
        if (flag_isr_calibration_range_current) //Toggle range calibration
        {
            flag_isr_calibration_range.compare_exchange_strong(flag_isr_calibration_range_current, false);
            handle_calibration_type = CalibrationState::Calibrating;
        }

        //Update calibration data if currently calibrating or flagged to change calibration state
        if (calibration_state != CalibrationState::None || handle_calibration_type != CalibrationState::None)
        {
            handle_calibration(handle_calibration_type);
        }

        //Update calibration indicators
        digitalWrite(PIN_OUT_INDICATE_ZERO, calibration_state == CalibrationState::Zeroing? HIGH : LOW);
        digitalWrite(PIN_OUT_INDICATE_CAL, calibration_state == CalibrationState::Calibrating? HIGH : LOW);

        //Is calibrating/zeroing; clear audio channel and return
        if (calibration_state != CalibrationState::None)
        {
            for (auto &c : audio_channels)
            {
                dacWrite(c, 0);
            }

            return;
        }

        const auto t = micros();

        //Get and update current volume
        const auto new_volume = analogRead(PIN_IN_VOLUME);
        volume->store(new_volume);

        //Get ratio to max volume
        const auto volume_scale = new_volume / (INPUT_MAX_VALUE * 1.);

        //Calculate next value in wave based on current frequency values

        //Split frequencies across channels
        const auto d = TOFSensorDriver::GetSensorCount() / sizeof(audio_channels);
        const auto r = TOFSensorDriver::GetSensorCount() % sizeof(audio_channels);

        //Max length of chord is sensor count / # of channels, + 1 if there's a remainder
        const auto chord_len = d + (r == 0? 0 : 1);
        double chord[chord_len];

        for (int i = 0, c = 0; c < sizeof(audio_channels); c++)
        {
            //Length of chord on this channel is sensor count / # of channels, + 1 if within remainder
            const auto channel_chord_len = d + ((r == 0 || c > r)? 0 : 1);

            //Set chord values
            for (int j = 0; j < channel_chord_len; j++)
            {
                chord[j] = frequencies[i++]->load();
            }

            //Calculate next value for wave and write to channel
            const auto wave_value = wave(t, volume_scale, chord, channel_chord_len);
            dacWrite(audio_channels[c], wave_value);
        }
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

    //Zero frequency
    if (sensor_value == 0)
        return 0;

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

CalibrationState handle_calibration(const CalibrationState toggle_state)
{
    //Get current state for calibration
    const auto current_calibration_state = calibration_state;

    //Determine what next state should be
    CalibrationState next_calibration_state = current_calibration_state;

    //Toggle state == None => no change
    if (toggle_state != CalibrationState::None)
    {
        if (current_calibration_state == CalibrationState::None) //Start calibration
        {
            next_calibration_state = toggle_state;
        }
        else if (current_calibration_state == toggle_state) //Stop calibration
        {
            next_calibration_state = CalibrationState::None;
            // Serial.println(sensors.ToString(true).c_str());
        }
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

void isr_timer()
{
    update_frequency(); //Update frequencies from sensor values on interval
}

void isr_zero()
{
    DEBOUNCE_MS(250)
    flag_isr_calibration_zero.store(true);
}

void isr_calibrate()
{
    DEBOUNCE_MS(250)
    flag_isr_calibration_range.store(true);
}
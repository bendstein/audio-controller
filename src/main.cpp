#include <Arduino.h>

#include "main.h"
#include "notes.h"

CalibrationState calibration_state = CalibrationState::None;

auto input_range_0_calibrating = new SensorRangeData(0, INPUT_MAX_VALUE, 0);
auto input_range_1_calibrating = new SensorRangeData(0, INPUT_MAX_VALUE, 0);

auto input_range_0 = new SensorRangeData(TOF_ZERO_DFT, TOF_MIN_DFT, TOF_MAX_DFT);
auto input_range_1 = new SensorRangeData(TOF_ZERO_DFT, TOF_MIN_DFT, TOF_MAX_DFT);

auto frequency_range_0 = new FrequencyRangeData(GetMusicalNoteFrequency(MusicalNote::A, 4), GetMusicalNoteFrequency(MusicalNote::A, 4));
auto frequency_range_1 = new FrequencyRangeData(GetMusicalNoteFrequency(MusicalNote::C, 4), GetMusicalNoteFrequency(MusicalNote::C, 4));

auto volume = new std::atomic_uint16_t(INPUT_MAX_VALUE);
auto frequency_0 = new std::atomic<double>(frequency_range_0->GetMin());
auto frequency_1 = new std::atomic<double>(frequency_range_1->GetMin());

hw_timer_t* timer = nullptr;

void setup() {
    Serial.begin(9600);

    try
    {
        // Serial.println("Start setup");

        //Setup pins
        pinMode(PIN_IN_TOF_0, PULLDOWN | INPUT);
        pinMode(PIN_IN_TOF_1, PULLDOWN | INPUT);
        pinMode(PIN_IN_ZERO, PULLDOWN | INPUT);
        pinMode(PIN_IN_CALIBRATE, PULLDOWN | INPUT);
        pinMode(PIN_IN_VOLUME, PULLUP | INPUT);

        pinMode(PIN_OUT_AUDIO_0, PULLDOWN | OUTPUT);
        pinMode(PIN_OUT_AUDIO_1, PULLDOWN | OUTPUT);
        pinMode(PIN_OUT_INDICATE_ZERO, PULLDOWN | OUTPUT);
        pinMode(PIN_OUT_INDICATE_CAL, PULLDOWN | OUTPUT);

        //Setup timer
        timer = timerBegin(0, 50, true);
        timerAttachInterrupt(timer, isr_timer, true);
        timerAlarmWrite(timer, TIMER_INTERVAL, true);

        timerAlarmEnable(timer);

        // Serial.println("End setup");
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

        // Serial.println("Start loop");
        const auto t = micros();

        //Get and update current volume
        const auto new_volume = analogRead(PIN_IN_VOLUME);
        volume->store(new_volume);

        const auto volume_scale = new_volume / (INPUT_MAX_VALUE * 1.);

        //Not calibrating

        //Calculate next value in wave based on current frequency values.
        const double current_frequency_0 = frequency_0->load();
        const double current_frequency_1 = frequency_1->load();

        //Calculate wave for each channel. Can put multiple frequencies in
        //each chord, don't need to be the same
        const double chord_0[] = { current_frequency_0, current_frequency_1 };
        const double chord_1[] = { current_frequency_0, current_frequency_1 };

        const auto wave_value_0 = wave(t, volume_scale, chord_0, sizeof(chord_0) / sizeof(chord_0[0]));
        const auto wave_value_1 = wave(t, volume_scale, chord_1, sizeof(chord_1) / sizeof(chord_1[0]));

        dacWrite(AUDIO_CHANNEL_0, wave_value_0);
        dacWrite(AUDIO_CHANNEL_1, wave_value_1);

        // Serial.println("End loop");
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

        if (chord_value <= 0)
            continue;

        const auto value = scale * sin(2 * PI * chord_value * ts);
        f += value;
        count += 1;
    }

    if (count > 0)
    {
        f = scale + f / count;
    }

    const auto v = static_cast<int32_t>(std::floor((DAC_MAX / 2.) * f));
    return static_cast<uint8_t>(std::max(0, std::min(DAC_MAX, v)));
}

void update_frequency()
{
    try
    {
        // Serial.println("Start update frequency");

        //Get current sensor range data if not calibrating
        const SensorRangeData* current_input_range_0 = nullptr;
        const SensorRangeData* current_input_range_1 = nullptr;

        if (calibration_state == CalibrationState::None)
        {
            current_input_range_0 = new SensorRangeData(input_range_0);
            current_input_range_1 = new SensorRangeData(input_range_1);
        }

        if (current_input_range_0 != nullptr)
        {
            const auto sensor_value_0 = readTOFPin(PIN_IN_TOF_0);
            const auto sensor_value_1 = readTOFPin(PIN_IN_TOF_1);

            //Calculate and assign the frequencies corresponding to the pin values
            frequency_0->store(calculate_frequency(sensor_value_0, current_input_range_0, frequency_range_0));
            frequency_1->store(calculate_frequency(sensor_value_1, current_input_range_1, frequency_range_1));
        }

        // Serial.println("End update frequency");
    }
    catch (std::exception& e)
    {
        Serial.println("An error occurred while updating frequency on timer.");
        Serial.println(e.what());
    }
}

double calculate_frequency(const uint16_t sensor_value, const SensorRangeData* sensor_range, const FrequencyRangeData* frequency_range)
{
    uint16_t sensor_zero, sensor_min, sensor_max;

    sensor_range->GetValues(sensor_zero, sensor_min, sensor_max);

    //Cut off value if at/below zero
    if (sensor_value <= sensor_zero)
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

CalibrationState handle_calibration()
{
    const auto current_calibration_state = calibration_state;

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

    //Handle change of state
    if (next_calibration_state != current_calibration_state)
    {
        switch (current_calibration_state)
        {
            case CalibrationState::Zeroing: //Done zeroing; commit values
                input_range_0->SetZero(input_range_0_calibrating->GetZero() + BUFFER_ZERO);
                input_range_1->SetZero(input_range_1_calibrating->GetZero() + BUFFER_ZERO);
                break;
            case CalibrationState::Calibrating: //Done calibrating; commit values
                input_range_0->UpdateRange(*input_range_0_calibrating);
                input_range_1->UpdateRange(*input_range_1_calibrating);
                break;
            case CalibrationState::None:
                break;
        }

        switch (next_calibration_state)
        {
            case CalibrationState::Zeroing: //Start zeroing; reset calibration values
                input_range_0_calibrating->SetZero(0);
                input_range_1_calibrating->SetZero(0);
                break;
            case CalibrationState::Calibrating: //Start calibrating; reset calibration values
                input_range_0_calibrating->UpdateRange(INPUT_MAX_VALUE, 0);
                input_range_1_calibrating->UpdateRange(INPUT_MAX_VALUE, 0);
                break;
            case CalibrationState::None:
                break;
        }
    }

    //Read sensor value and update zeroes/range
    const auto sensor_value_0 = next_calibration_state == CalibrationState::None?
        0
    : readTOFPin(PIN_IN_TOF_0);

    const auto sensor_value_1 = next_calibration_state == CalibrationState::None?
        0
        : readTOFPin(PIN_IN_TOF_1);

    switch (next_calibration_state)
    {
        case CalibrationState::Zeroing: //Update zeroes
            input_range_0_calibrating->UpdateZeroIfGreater(sensor_value_0);
            input_range_1_calibrating->UpdateZeroIfGreater(sensor_value_1);
            break;
        case CalibrationState::Calibrating: //Update input ranges
            input_range_0_calibrating->UpdateRangeIfMinMax(sensor_value_0);
            input_range_1_calibrating->UpdateRangeIfMinMax(sensor_value_1);
            break;
        case CalibrationState::None:
            break;
    }

    //Update and return new calibration state
    calibration_state = next_calibration_state;
    return next_calibration_state;
}


uint16_t readTOFPin(const uint8_t pin)
{
    const auto sensor_value = analogRead(pin);

    return TOF_ROUNDING * static_cast<uint16_t>(std::floor(sensor_value / (TOF_ROUNDING * 1.)));
}

void isr_timer()
{
    update_frequency();
}
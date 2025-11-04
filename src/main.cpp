#include <Arduino.h>

#include "main.h"
#include "common.h"
#include "notes.h"
#include "rotary.h"
#include "sensor_array_driver.h"

//Attached sensors
static constexpr uint8_t sensor_select_pins[SensorArrayDriver::SELECT_PIN_COUNT] = { PIN_OUT_SENSOR_SELECT_0, PIN_OUT_SENSOR_SELECT_1, PIN_OUT_SENSOR_SELECT_2 };
static auto sensors = SensorArrayDriver(PIN_IN_SENSOR, sensor_select_pins);

static Rotary* rotary_encoder;

//Range of output frequencies to map sensors to
static const FrequencyRangeData frequency_ranges[SensorArrayDriver::MAX_SENSOR_COUNT] = {
    FrequencyRangeData(GetMusicalNoteFrequency(MusicalNote::C, 4), GetMusicalNoteFrequency(MusicalNote::C, 4)),
    FrequencyRangeData(GetMusicalNoteFrequency(MusicalNote::C, 4), GetMusicalNoteFrequency(MusicalNote::C, 4)),
    FrequencyRangeData(GetMusicalNoteFrequency(MusicalNote::C, 4), GetMusicalNoteFrequency(MusicalNote::C, 4)),
    FrequencyRangeData(GetMusicalNoteFrequency(MusicalNote::C, 4), GetMusicalNoteFrequency(MusicalNote::C, 4)),
    FrequencyRangeData(GetMusicalNoteFrequency(MusicalNote::C, 4), GetMusicalNoteFrequency(MusicalNote::C, 4)),
    FrequencyRangeData(GetMusicalNoteFrequency(MusicalNote::C, 4), GetMusicalNoteFrequency(MusicalNote::C, 4)),
    FrequencyRangeData(GetMusicalNoteFrequency(MusicalNote::C, 4), GetMusicalNoteFrequency(MusicalNote::C, 4)),
    FrequencyRangeData(GetMusicalNoteFrequency(MusicalNote::C, 4), GetMusicalNoteFrequency(MusicalNote::C, 4))
};

//The audio frequency to play based on the last-read input from sensors
static std::atomic<double> frequencies[SensorArrayDriver::MAX_SENSOR_COUNT] {};

static constexpr uint8_t audio_channels[] = { AUDIO_CHANNEL_0, AUDIO_CHANNEL_1 };

//Value (out of INPUT_MAX_VALUE) representing to volume of the output audio
static auto volume = new std::atomic_uint16_t(INPUT_MAX_VALUE);

//Amount of time (in microseconds) to delay between iterations when looping over sensors
static auto sensor_iterate_delay = new std::atomic_uint32_t(SENSOR_ITERATE_DELAY_DFT);

//Flags
std::atomic_bool flag_debugging {};
std::atomic_bool flag_update_frequencies {};

void setup() {
    Serial.begin(9600);
    Serial.println();

    try
    {
        //Set initial frequency values to bottom of respective ranges
        for (int i = 0; i < SensorArrayDriver::MAX_SENSOR_COUNT; i++)
        {
            frequencies[i].store(frequency_ranges[i].GetMin());
        }

        //Setup pins
        pinMode(PIN_OUT_DEBUG, OUTPUT);
        pinMode(PIN_IN_TOGGLE_DEBUG, PULLDOWN | INPUT);
        pinMode(PIN_IN_AUDIO_VOLUME, PULLDOWN | INPUT);

        pinMode(PIN_OUT_AUDIO_0, OUTPUT);
        pinMode(PIN_OUT_AUDIO_1, OUTPUT);

        pinMode(PIN_IN_SENSOR, PULLDOWN | INPUT);
        for (const auto sensor_select_pin : sensor_select_pins)
        {
            pinMode(sensor_select_pin, OUTPUT);
            digitalWrite(sensor_select_pin, LOW);
        }

        //Write some initial debug info
        print_debug_info();

        //Setup rotary encoder
        rotary_encoder = new Rotary(PIN_IN_ROT_CYCLE_RATE_A, PIN_IN_ROT_CYCLE_RATE_B, rotary_encoder_callback);

        //Setup timer
        const auto timer = timerBegin(0, 50, true);
        timerAttachInterrupt(timer, isr_timer, true);
        timerAlarmWrite(timer, TIMER_INTERVAL, true);
        timerAlarmEnable(timer);

        //Attach interrupts
        attachInterrupt(digitalPinToInterrupt(PIN_IN_TOGGLE_DEBUG), isr_debug_toggle, RISING);
    }
    catch (std::exception& e)
    {
        Serial.println("An error occurred during setup.");
        Serial.println(e.what());
    }
}

void loop()
{
    static uint64_t n = 0;

    try
    {
        //Read flags
        auto should_update_frequencies = flag_update_frequencies.load();
        const auto is_debugging = flag_debugging.load();

        //Update frequencies if flag set, and unset flag
        flag_update_frequencies.compare_exchange_weak(should_update_frequencies, false);

        if (should_update_frequencies)
        {
            update_frequency();
        }

        n = (n + 1) % std::numeric_limits<uint64_t>::max();

        //Periodically print debug info if debug flag is set
        if (is_debugging && n % 15000 == 0)
        {
            print_debug_info();
        }

        const auto t = micros();

        //Get and update current volume
        const auto new_volume = analogRead(PIN_IN_AUDIO_VOLUME);
        volume->store(new_volume);

        //Get ratio to max volume
        const auto volume_scale = new_volume / (INPUT_MAX_VALUE * 1.);

        //Calculate next value in wave based on current frequency values

        //Split frequencies across channels
        const auto d = sensors.EffectiveMaxSensorCount() / sizeof(audio_channels);
        const auto r = sensors.EffectiveMaxSensorCount() % sizeof(audio_channels);

        //Max length of chord is sensor count / # of channels, + 1 if there's a remainder
        const auto chord_len = d + (r == 0? 0 : 1);
        double chord[chord_len];

        for (int i = 0, c = 0; c < sizeof(audio_channels); c++)
        {
            //Length of chord on this channel is sensor count / # of channels, + 1 if within remainder
            const auto channel_chord_len = d + ((r == 0 || c > r)? 0 : 1);

            //Set chord values
            for (double &j : chord)
            {
                j = frequencies[i++].load();
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
        //Update frequencies based on sensor values
        for (auto i = 0; i < sensors.EffectiveMaxSensorCount(); i++)
        {
            const auto iterate_delay = sensor_iterate_delay->load();

            if (iterate_delay > 0)
                delay(iterate_delay);

            frequencies[i].store(calculate_frequency(i, frequency_ranges[i]));
        }
    }
    catch (std::exception& e)
    {
        Serial.println("An error occurred while updating frequency on timer.");
        Serial.println(e.what());
    }
}

double calculate_frequency(const int sensor, const FrequencyRangeData &frequency_range)
{
    const auto sensor_distance_cm = sensors.ReadSensorDistance(sensor);

    //Distance of 0 indicates that sensor isn't attached
    if (sensor_distance_cm == 0)
        return 0;

    //Get ratio into range that sensor value is at
    const auto ratio = 1.; //TODO

    //Map ratio to frequency
    double frequency_min, frequency_max;
    frequency_range.GetValues(frequency_min, frequency_max);

    const auto calculated_frequency = ((frequency_max - frequency_min) * ratio) + frequency_min;

    return calculated_frequency;
}

void print_debug_info()
{
    //Skip if not debugging
    if (!flag_debugging.load())
        return;

    std::stringstream stream {};

    const auto vol = volume->load();
    const auto vol_percent = (100. * vol) / INPUT_MAX_VALUE;
    const auto iterate_delay = sensor_iterate_delay->load();

    stream << std::fixed << std::showpoint << std::setprecision(2)
        << "Volume: " << vol
        << " (" << vol_percent << "%)" << std::endl
        << "Iterate delay: " << iterate_delay << "ms" << std::endl
        << "Frequencies:" << std::endl;

    sensors.WriteToStringStream(stream);

    stream << std::endl;

    Serial.println(stream.str().c_str());
}

void isr_timer()
{
    //Set flag to update frequencies from sensor values on interval
    flag_update_frequencies.store(true);
}

void isr_debug_toggle()
{
    auto current_debug_flag = flag_debugging.load();

    DEBOUNCE_MS(250)

    //Toggle debug flag if not somehow changed elsewhere
    const auto new_debug_flag = !current_debug_flag;

    if (flag_debugging.compare_exchange_weak(current_debug_flag, new_debug_flag))
    {
        digitalWrite(PIN_OUT_DEBUG, new_debug_flag? HIGH : LOW);
    }
}

void rotary_encoder_callback(const bool clockwise)
{
    static constexpr uint32_t max_safe_delay = std::numeric_limits<uint32_t>::max() - SENSOR_ITERATE_DELAY_STEP;

    const auto current_iterate_delay = sensor_iterate_delay->load();
    uint32_t new_iterate_delay = 0;

    //Update iterate delay, clamping to valid range
    if (clockwise)
    {
        if (current_iterate_delay >= max_safe_delay)
        {
            new_iterate_delay = std::numeric_limits<uint32_t>::max();
        }
        else
        {
            new_iterate_delay = current_iterate_delay + SENSOR_ITERATE_DELAY_STEP;
        }
    }
    else
    {
        if (current_iterate_delay <= SENSOR_ITERATE_DELAY_STEP)
        {
            new_iterate_delay = 0;
        }
        else
        {
            new_iterate_delay = current_iterate_delay - SENSOR_ITERATE_DELAY_STEP;
        }
    }

    sensor_iterate_delay->store(new_iterate_delay);
}
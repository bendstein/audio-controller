//
// Created by bendstein on 9/30/2025.
//

#ifndef AUDIO_CONTROLLER_MAIN_H
#define AUDIO_CONTROLLER_MAIN_H

#include <atomic>

#define PIN_IN_TOF_0 A2                     //Input pin for first TOF sensor
#define PIN_IN_TOF_1 A3                     //Input pin for second TOF sensor
#define PIN_IN_ZERO A6                      //Input pin to zero sensors
#define PIN_IN_CALIBRATE A7                 //Input pin to calibrate range for sensors
#define PIN_IN_VOLUME A10                   //Input pin to adjust audio volume

#define PIN_OUT_AUDIO_0 A1                  //Pin corresponding to first audio output
#define PIN_OUT_AUDIO_1 A0                  //Pin corresponding to second audio output
#define PIN_OUT_INDICATE_ZERO A5            //Output pin indicating whether currently zero-ing sensors
#define PIN_OUT_INDICATE_CAL GPIO_NUM_21    //Output pin indicating whether currently calibrating sensors

#define AUDIO_CHANNEL_0 DAC1                //DAC channel for PIN_OUT_AUDIO_0
#define AUDIO_CHANNEL_1 DAC2                //DAC channel for PIN_OUT_AUDIO_1

#define INPUT_MAX_VALUE 4096                //Max value that can be received by analogRead()
#define BUFFER_ZERO 48                      //Add buffer when zeroing to help account for noise
#define DAC_MAX 255                         //Max output value for a DAC channel
#define TIMER_INTERVAL 50000                //Frequency at which update timer should trigger
#define TOF_ROUNDING 16                     //Round input values from TOF pins to the previous multiple of this value
#define TOF_ZERO_DFT 1000                   //Default zero for TOF input before calibration
#define TOF_MIN_DFT 1150                    //Default min value in range for TOF input before calibration
#define TOF_MAX_DFT INPUT_MAX_VALUE         //Default max value in range for TOF input before calibration

/**
 * Whether sensors are being calibrated, and if so,
 * which type
 */
enum CalibrationState
{
    None = 0,
    Zeroing = 1,
    Calibrating = 2
};

/**
 * Data representing the valid range of values that a sensor
 * can produce.
 */
class SensorRangeData
{
    /**
     * The max value that the sensor produces when not being interacted
     * with.
     */
    std::atomic_uint16_t zero;

    /**
     * The range of values that the sensor can produce when it is
     * being interacted with. Each is a 16-bit, unsigned integer,
     * with the max as the most-significant 16 bits and the min
     * as the least-significant 16 bits.
     */
    std::atomic_uint32_t minmax;
public:
    /**
     *
     * @param zero The max value that the sensor produces when not being
     *  interacted with.
     * @param min The min value that the sensor can produce when it is
     *  being interacted with.
     * @param max The max value that the sensor can produce when it is
     *  being interacted with.
     */
    SensorRangeData(const uint16_t zero, const uint16_t min, const uint16_t max)
    {
        this->zero = zero;
        this->minmax = (max << 16) | min;
    }

    /**
     * Default constructor, with all values initialized to 0
     */
    SensorRangeData() : SensorRangeData(0, 0, 0) {}

    /**
     * Copy constructor
     * @param other The SensorRangeData to copy values from
     */
    explicit SensorRangeData(const SensorRangeData* other) : SensorRangeData()
    {
        Update(*other);
    }

    /**
     * @return The sensor's zero
     */
    uint16_t GetZero() const
    {
        return this->zero.load();
    }

    /**
     * @param new_zero The sensor's new zero
     */
    void SetZero(const uint16_t new_zero)
    {
        this->zero.store(new_zero);
    }

    /**
     * @return The sensor's min
     */
    uint16_t GetMin() const
    {
        //Min is the bottom 16 bits of minmax
        return this->minmax.load() & 0xFFFF;
    }

    /**
     * @param new_min The sensor's new min
     */
    void SetMin(const uint16_t new_min)
    {
        //Min is the bottom 16 bits of minmax,
        //so make sure top 16 bits are preserved
        //during update
        this->minmax &= new_min | (0xFFFF << 16);
    }

    /**
     * @return The sensor's max
     */
    uint16_t GetMax() const
    {
        //Max is the top 16 bits of minmax
        return this->minmax.load() >> 16;
    }

    /**
     * @param new_max The sensor's new max
     */
    void SetMax(const uint16_t new_max)
    {
        //Max is the top 16 bits of minmax,
        //so make sure bottom 16 bits are
        //preserved during update
        this->minmax &= new_max | 0xFFFF;
    }

    /**
     * @return A new instance of Self with the same
     *  values as this
     */
    SensorRangeData* GetClone() const
    {
        const uint16_t value_zero = this->zero.load();
        const uint32_t value_minmax = this->minmax.load();
        const uint16_t value_min = value_minmax & 0xFFFF;
        const uint16_t value_max = value_minmax >> 16;

        return new SensorRangeData(value_zero, value_min, value_max);
    }

    /**
     * Load data into reference params
     * @param value_zero The sensor's zero
     * @param value_min The sensor's min
     * @param value_max The sensor's max
     */
    void GetValues(uint16_t& value_zero, uint16_t& value_min, uint16_t& value_max) const
    {
        value_zero = this->zero.load();
        const uint32_t value_minmax = this->minmax.load();
        value_min = value_minmax & 0xFFFF;
        value_max = value_minmax >> 16;
    }

    /**
     * Load range data into reference params
     * @param value_min The sensor's min
     * @param value_max The sensor's max
     */
    void GetRange(uint16_t& value_min, uint16_t& value_max) const
    {
        const uint32_t value_minmax = this->minmax.load();
        value_min = value_minmax & 0xFFFF;
        value_max = value_minmax >> 16;
    }

    /**
     * Update values
     * @param new_zero The sensor's new zero
     * @param new_min The sensor's new max
     * @param new_max The sensor's new min
     */
    void Update(const uint16_t new_zero, const uint16_t new_min, const uint16_t new_max)
    {
        this->zero.store(new_zero);
        this->minmax.store(new_min | (new_max << 16));
    }

    /**
     * Update values from another instance
     * @param other The instance to update from
     */
    void Update(const SensorRangeData& other)
    {
        uint16_t value_zero, value_min, value_max;
        other.GetValues(value_zero, value_min, value_max);

        Update(value_zero, value_min, value_max);
    }

    /**
     * Update range values
     * @param new_min The sensor's new max
     * @param new_max The sensor's new min
     */
    void UpdateRange(const uint16_t new_min, const uint16_t new_max)
    {
        this->minmax.store(new_min | (new_max << 16));
    }

    /**
     * Update range values from another instance of Self
     * @param other The instance to update from
     */
    void UpdateRange(const SensorRangeData& other)
    {
        uint16_t value_min, value_max;
        other.GetRange(value_min, value_max);

        UpdateRange(value_min, value_max);
    }

    /**
     * Update the current value of zero if the new zero
     * is greater than it.
     * @param new_zero The sensor's new min (if the condition is satisfied)
     * @todo Can this livelock?
     */
    void UpdateZeroIfGreater(const uint16_t new_zero)
    {
        //Update the current value for zero with the new value,
        //retrying if the current value changed during the processed
        while (true)
        {
            auto current_zero = this->zero.load();

            if (new_zero > current_zero)
            {
                if (this->zero.compare_exchange_strong(current_zero, new_zero))
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
     * @param new_value The sensor's new min/max (if this condition is satisfied)
     * @todo Can this livelock?
     */
    void UpdateRangeIfMinMax(const uint16_t new_value)
    {
        //Update the current values for minmax with the new value,
        //retrying if the current value changed during the processed
        while (true)
        {
            auto current_minmax = this->minmax.load();
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

    /**
     * @return A string describing the contained data.
     */
    std::string ToString() const
    {
        uint16_t value_zero, value_min, value_max;
        this->GetValues(value_zero, value_min, value_max);
        const uint32_t value_minmax = this->minmax.load();

        auto text = std::string(254, '\0');
        sprintf(&text[0], "SensorRangeData|z: %d, r: %d - %d (%d)", value_zero, value_min, value_max, value_minmax);

        return text;
    }
};

/**
 * Data representing a range of frequencies that can be produced
 */
class FrequencyRangeData
{
    /**
     * The minimum frequency in the range
     */
    std::atomic<double> min;

    /**
     * The maximum frequency in the range
     */
    std::atomic<double> max;

public:
    /**
     * @param min The minimum frequency in the range
     * @param max The maximum frequency in the range
     */
    FrequencyRangeData(const double min, const double max)
    {
        this->min = min;
        this->max = max;
    }

    /**
     * Default constructor, with all values initialized to 0
     */
    FrequencyRangeData() : FrequencyRangeData(0, 0) {}

    /**
     * Copy constructor
     * @param other The FrequencyRangeData to copy values from
     */
    explicit FrequencyRangeData(const FrequencyRangeData* other) : FrequencyRangeData()
    {
        Update(*other);
    }

    /**
     * @return The range's min
     */
    double GetMin() const
    {
        return this->min.load();
    }

    /**
     * @param new_min The range's new min
     */
    void SetMin(const double new_min)
    {
        this->min.store(new_min);
    }

    /**
     * @return The range's max
     */
    double GetMax() const
    {
        return this->max.load();
    }

    /**
     * @param new_max The range's new max
     */
    void SetMax(const double new_max)
    {
        this->max.store(new_max);
    }

    /**
     * @return A new instance of FrequencyRangeData with the same
     *  values as this
     */
    FrequencyRangeData* GetClone() const
    {
        return new FrequencyRangeData(this->min, this->max);
    }

    /**
     * Load data into reference params
     * @param value_min The range's min
     * @param value_max The range's max
     */
    void GetValues(double& value_min, double& value_max) const
    {
        value_min = this->min;
        value_max = this->max;
    }

    /**
     * Update values
     * @param new_min The range's new min
     * @param new_max The range's new max
     */
    void Update(const double& new_min, const double& new_max)
    {
        this->min = new_min;
        this->max = new_max;
    }

    /**
     * Update values from another instance of Self
     * @param other The instance to copy values from
     */
    void Update(const FrequencyRangeData& other)
    {
        double value_min, value_max;
        other.GetValues(value_min, value_max);

        Update(value_min, value_max);
    }

    /**
     * @return A string describing the contained data.
     */
    std::string ToString() const
    {
        double value_min, value_max;
        this->GetValues(value_min, value_max);

        auto text = std::string(254, '\0');
        sprintf(&text[0], "FrequencyRangeData|r: %.2f - %.2f", value_min, value_max);

        return text;
    }
};

/**
 * Read the analog value from a pin with an attached sensor
 * @param pin The pin to read from
 * @return The value of the sensor
 */
uint16_t readTOFPin(uint8_t pin);

/**
 * Get the next audio value to output
 * @param t The current time in microseconds
 * @param scale Coefficient to scale the output wave, to control volume
 * @param chord The component frequencies, in hz, of the wave
 * @param chord_len The length of the chord array
 * @return The next value in the output wave, from 0 to 255
 */
uint8_t wave(uint64_t t, double scale, const double chord[], uint8_t chord_len);

/**
 * Get the new frequency for a given sensor
 * @param sensor_value The value from the sensor
 * @param sensor_range Calibration/range data for the given sensor
 * @param frequency_range The range of potential output frequencies
 * @return The frequency corresponding to the given sensor value
 */
double calculate_frequency(uint16_t sensor_value, const SensorRangeData* sensor_range, const FrequencyRangeData* frequency_range);

/**
 * Update current frequency values based on sensors
 */
void update_frequency();

/**
 * Handle calibrating sensors, as well as
 * changes in calibration state.
 * @return The new calibration state
 */
CalibrationState handle_calibration();

/**
 * Callback for timer interrupt
 */
void isr_timer();

#endif //AUDIO_CONTROLLER_MAIN_H
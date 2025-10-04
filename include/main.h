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

enum CalibrationState
{
    None = 0,
    Zeroing = 1,
    Calibrating = 2
};

class SensorRangeData
{
    std::atomic_uint16_t zero;
    std::atomic_uint32_t minmax;
public:
    SensorRangeData(const uint16_t zero, const uint16_t min, const uint16_t max)
    {
        this->zero = zero;
        this->minmax = (max << 16) | min;
    }

    SensorRangeData() : SensorRangeData(0, 0, 0) {}

    explicit SensorRangeData(const SensorRangeData* other) : SensorRangeData()
    {
        Update(*other);
    }

    uint16_t GetZero() const
    {
        return this->zero.load();
    }

    void SetZero(const uint16_t new_zero)
    {
        this->zero.store(new_zero);
    }

    uint16_t GetMin() const
    {
        return this->minmax.load() & 0xFFFF;
    }

    void SetMin(const uint16_t new_min)
    {
        this->minmax &= new_min | (0xFFFF << 16);
    }

    uint16_t GetMax() const
    {
        return this->minmax.load() >> 16;
    }

    void SetMax(const uint16_t new_max)
    {
        this->minmax &= new_max | 0xFFFF;
    }

    SensorRangeData* GetClone() const
    {
        const uint16_t value_zero = this->zero.load();
        const uint32_t value_minmax = this->minmax.load();
        const uint16_t value_min = value_minmax & 0xFFFF;
        const uint16_t value_max = value_minmax >> 16;

        return new SensorRangeData(value_zero, value_min, value_max);
    }

    void GetValues(uint16_t& value_zero, uint16_t& value_min, uint16_t& value_max) const
    {
        value_zero = this->zero.load();
        const uint32_t value_minmax = this->minmax.load();
        value_min = value_minmax & 0xFFFF;
        value_max = value_minmax >> 16;

    }

    void GetRange(uint16_t& value_min, uint16_t& value_max) const
    {
        const uint32_t value_minmax = this->minmax.load();
        value_min = value_minmax & 0xFFFF;
        value_max = value_minmax >> 16;
    }

    void Update(const uint16_t new_zero, const uint16_t new_min, const uint16_t new_max)
    {
        this->zero.store(new_zero);
        this->minmax.store(new_min | (new_max << 16));
    }

    void Update(const SensorRangeData& other)
    {
        uint16_t value_zero, value_min, value_max;
        other.GetValues(value_zero, value_min, value_max);

        Update(value_zero, value_min, value_max);
    }

    void UpdateRange(const uint16_t new_min, const uint16_t new_max)
    {
        this->minmax.store(new_min | (new_max << 16));
    }

    void UpdateRange(const SensorRangeData& other)
    {
        uint16_t value_min, value_max;
        other.GetRange(value_min, value_max);

        UpdateRange(value_min, value_max);
    }

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

class FrequencyRangeData
{
    std::atomic<double> min;
    std::atomic<double> max;

public:
    FrequencyRangeData(const double min, const double max)
    {
        this->min = min;
        this->max = max;
    }

    FrequencyRangeData() : FrequencyRangeData(0, 0) {}

    explicit FrequencyRangeData(const FrequencyRangeData* other) : FrequencyRangeData()
    {
        Update(*other);
    }

    double GetMin() const
    {
        return this->min.load();
    }

    void SetMin(const double new_min)
    {
        this->min.store(new_min);
    }

    double GetMax() const
    {
        return this->max.load();
    }

    void SetMax(const double new_max)
    {
        this->max.store(new_max);
    }

    FrequencyRangeData* GetClone() const
    {
        return new FrequencyRangeData(this->min, this->max);
    }

    void GetValues(double& value_min, double& value_max) const
    {
        value_min = this->min;
        value_max = this->max;
    }

    void Update(const double& new_min, const double& new_max)
    {
        this->min = new_min;
        this->max = new_max;
    }

    void Update(const FrequencyRangeData& other)
    {
        double value_min, value_max;
        other.GetValues(value_min, value_max);

        Update(value_min, value_max);
    }

    std::string ToString() const
    {
        double value_min, value_max;
        this->GetValues(value_min, value_max);

        auto text = std::string(254, '\0');
        sprintf(&text[0], "FrequencyRangeData|r: %.2f - %.2f", value_min, value_max);

        return text;
    }
};

uint16_t readTOFPin(uint8_t pin);
uint8_t wave(uint64_t t, double scale, const double chord[], uint8_t chord_len);
double calculate_frequency(uint16_t sensor_value, const SensorRangeData* sensor_range, const FrequencyRangeData* frequency_range);
void update_frequency();
CalibrationState handle_calibration();

void isr_timer();

#endif //AUDIO_CONTROLLER_MAIN_H
//
// Created by bendstein on 10/5/2025.
//

#ifndef AUDIO_CONTROLLER_FREQUENCY_RANGE_DATA_H
#define AUDIO_CONTROLLER_FREQUENCY_RANGE_DATA_H

/**
 * Data representing a range of frequencies that can be produced
 */
class FrequencyRangeData
{
    /**
     * The minimum frequency in the range
     */
    std::atomic<double> min {};

    /**
     * The maximum frequency in the range
     */
    std::atomic<double> max {};

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
    FrequencyRangeData(const FrequencyRangeData& other) : FrequencyRangeData()
    {
        Update(other);
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

#endif //AUDIO_CONTROLLER_FREQUENCY_RANGE_DATA_H
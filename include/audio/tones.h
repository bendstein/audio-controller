//
// Created by bendstein on 1/28/2026.
//

#ifndef AUDIO_CONTROLLER_TONES_H
#define AUDIO_CONTROLLER_TONES_H
#include <cmath>

#include "app_common.h"
#include "musical_note_data.h"
#include "musical_note.h"

constexpr double FREQUENCY_A4 = 440; //Frequency, in hz, for A in octave 4
constexpr double FREQUENCY_HZ_EPS = 0.001; //Consider fr within this amount to be equivalent

[[nodiscard]] constexpr double musical_note_freq_hz(const musical_note note, const uint8_t octave)
{
    if (octave == 0) //Special case; if octave is 0, return 0
        return 0.;

    constexpr auto semitone_a4 = static_cast<uint8_t>(musical_note::A) + (4 * static_cast<uint8_t>(musical_note::MAX));
    const auto semitone = static_cast<uint8_t>(note) + (octave * static_cast<uint8_t>(musical_note::MAX));

    return FREQUENCY_A4 * std::pow(2, static_cast<double>(semitone - semitone_a4) / (static_cast<uint8_t>(musical_note::MAX) * 1.));
}

[[nodiscard]] constexpr double hz_to_megahz(const double hz) { return hz / US_PER_SECOND; }

[[nodiscard]] constexpr bool check_frequency_equivalency(const double a, const double b) { return std::abs(a - b) <= FREQUENCY_HZ_EPS; }

struct musical_note_tone
{
    musical_note note;
    uint8_t octave;

    [[nodiscard]] bool is_zero() const { return octave == 0; }
    [[nodiscard]] bool is_invalid() const { return octave == 0xFF; }
    [[nodiscard]] bool is_zero_or_invalid() const { return octave == 0 || octave == 0xFF; }
    [[nodiscard]] double frequency_hz() const { return is_invalid()? 0 : musical_note_freq_hz(note, octave); }
    [[nodiscard]] const char* name(const bool prefer_flat = false) const { return get_note_name(note, octave, prefer_flat); }
    [[nodiscard]] const char* name_no_octave(const bool prefer_flat = false) const { return get_note_name(note, prefer_flat); }
    [[nodiscard]] uint32_t get_period(uint32_t sample_rate) const;
    [[nodiscard]] double get_periodd(uint32_t sample_rate) const;

    bool operator==(const musical_note_tone& other) const { return check_frequency_equivalency(frequency_hz(), other.frequency_hz()); }
    bool operator==(const double other_hz) const { return check_frequency_equivalency(frequency_hz(), other_hz); }

    [[nodiscard]] static consteval musical_note_tone create_zero() { return musical_note_tone(musical_note::C, 0); }
    [[nodiscard]] static consteval musical_note_tone create_invalid() { return musical_note_tone(musical_note::C, 0xFF); }
    [[nodiscard]] static uint32_t get_period(double frequency_hz, uint32_t sample_rate);
    [[nodiscard]] static double get_periodd(double frequency_hz, uint32_t sample_rate);
};

struct musical_note_tone_volume
{
    musical_note_tone tone;
    uint8_t volume;

    [[nodiscard]] bool is_zero() const { return volume == 0 || tone.is_zero(); }
    [[nodiscard]] bool is_invalid() const { return tone.is_invalid(); }
    [[nodiscard]] bool is_zero_or_invalid() const { return volume == 0 || tone.is_invalid(); }
    [[nodiscard]] uint8_t sine_table_value_at(uint64_t index) const;

    bool operator==(const musical_note_tone_volume& other) const { return tone == other.tone && volume == other.volume; }

    [[nodiscard]] static consteval musical_note_tone_volume create_zero() { return musical_note_tone_volume(musical_note_tone::create_zero(), 0); }
    [[nodiscard]] static consteval musical_note_tone_volume create_invalid() { return musical_note_tone_volume(musical_note_tone::create_invalid(), 0); }
};

struct frequency_volume
{
    double frequency_hz;
    uint8_t volume;

    frequency_volume() = default;
    frequency_volume(const double frequency_hz, const uint8_t volume) : frequency_hz(frequency_hz), volume(volume) {}
    frequency_volume(const musical_note_tone& tone, const uint8_t volume) : frequency_hz(tone.frequency_hz()), volume(volume) {}
    frequency_volume(const musical_note_tone& tone, const uint8_t volume, const double vibrato) : frequency_volume(tone, volume)
    {
        //Apply vibrato as ratio between tone and next/prev note
        if (vibrato > 0 && vibrato != 1 && vibrato < 2 && !tone.is_zero_or_invalid())
        {
            if (vibrato > 1) //Vibrato between self and next note
            {
                bool octave_change;
                const auto other_note = next_note(tone.note, &octave_change);
                const uint8_t other_octave = octave_change? tone.octave + 1 : tone.octave;

                if (const auto other_tone = musical_note_tone(other_note, other_octave);
                    !other_tone.is_zero_or_invalid())
                {
                    const auto diff = other_tone.frequency_hz() - tone.frequency_hz();
                    const auto add = (vibrato - 1) * diff;
                    const auto new_frequency = tone.frequency_hz() + add;

                    this->frequency_hz = new_frequency;
                }
            }
            else //Vibrato between self and prev note
            {
                bool octave_change;
                const auto other_note = next_note(tone.note, &octave_change);
                const uint8_t other_octave = octave_change? tone.octave - 1 : tone.octave;

                if (const auto other_tone = musical_note_tone(other_note, other_octave);
                    !other_tone.is_zero_or_invalid())
                {
                    const auto diff = tone.frequency_hz() - other_tone.frequency_hz();
                    const auto sub = vibrato * diff;
                    const auto new_frequency = tone.frequency_hz() - sub;

                    this->frequency_hz = new_frequency;
                }
            }
        }
    }
    explicit frequency_volume(const musical_note_tone_volume& data) : frequency_volume(data.tone, data.volume) {}
    explicit frequency_volume(const musical_note_tone_volume& data, const double vibrato) : frequency_volume(data.tone, data.volume, vibrato) {}

    [[nodiscard]] bool is_zero() const { return check_frequency_equivalency(frequency_hz, 0) || volume == 0; }

    bool operator==(const frequency_volume& other) const { return check_frequency_equivalency(frequency_hz, other.frequency_hz) && volume == other.volume; }
    [[nodiscard]] static consteval frequency_volume create_zero() { return { 0, 0 }; }
};

#endif //AUDIO_CONTROLLER_TONES_H
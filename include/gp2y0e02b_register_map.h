//
// Created by bendstein on 12/22/2025.
//

#ifndef AUDIO_CONTROLLER_GP2Y0E02B_REGISTER_MAP_H
#define AUDIO_CONTROLLER_GP2Y0E02B_REGISTER_MAP_H
#include <cstdint>

namespace gp2y0e02b
{
    enum register_map_tag : uint8_t
    {
        UNKNOWN,
        HOLD_BIT,
        MAXIMUM_EMITTING_PULSE_WIDTH,
        SPOT_SYMMETRY_THRESHOLD,
        SIGNAL_INTENSITY_THRESHOLD,
        MAXIMUM_SPOT_SIZE_THRESHOLD,
        MINIMUM_SPOT_SIZE_THRESHOLD,
        SHIFT_BIT,
        MEDIAN_FILTER,
        SRAM_ACCESS,
        DISTANCE_11_THRU_4,
        DISTANCE_3_THRU_0,
        AE_15_THRU_8,
        AE_7_THRU_0,
        AG_7_THRU_0,
        COVER_COMPENSATION_5_THRU_0,
        COVER_COMPENSATION_10_THRU_6,
        COVER_COMPENSATION_ENABLE_BIT,
        READ_OUT_IMAGE_SENSOR_DATA,
        SIGNAL_ACCUMULATION_NUMBER,
        SIGNAL_INTENSITY_ENABLE_BIT,
        MAXIMUM_SPOT_SIZE_ENABLE_BIT,
        MINIMUM_SPOT_SIZE_ENABLE_BIT,
        SPOT_SYMMETRY_ENABLE_BIT,
        EFUSE_TARGET_ADDRESS,
        EFUSE_READ_OUT_AND_ENABLE_BIT,
        EFUSE_BIT_NUMBER_AND_BANK_ASSIGN,
        EFUSE_PROGRAM_ENABLE_BIT,
        EFUSE_PROGRAM_DATA,
        ACTIVE_STAND_BY_STATE_CONTROL,
        CLOCK_SELECT,
        SOFTWARE_RESET,
        BANK_SELECT,
        RIGHT_EDGE_COORDINATE,
        LEFT_EDGE_COORDINATE,
        PEAK_COORDINATE,
    };

    struct register_map_entry
    {
        register_map_tag tag;
        uint8_t address;
        union RegisterData
        {
            uint8_t data;
            enum MaxPulseWidth : uint8_t {
                us_40 = 0x03,
                us_80 = 0x04,
                us_160 = 0x05,
                us_240 = 0x06,
                us_320 = 0x07,
            } data_maximum_emitting_pulse_width: 2;
            uint8_t data_spot_symmetry_threshold: 4;
            uint8_t data_signal_intensity_threshold: 6;
            uint8_t data_maximum_spot_size_threshold: 7;
            uint8_t data_minimum_spot_size_threshold: 6;
            enum ShiftBit : uint8_t {
                max_128_cm = 0x01,
                max_64_cm = 0x02
            } data_shift_bit: 2;
            struct DataMedianFilter
            {
                uint8_t __dummy: 5;
                enum MedianFilter : uint8_t {
                    value_7 = 0x00,
                    value_5 = 0x10,
                    value_9 = 0x20,
                    value_1 = 0x30
                } data_medium_filter: 2;
            };
            uint8_t data_access_sram: 4;
            
        };
    };
}

#endif //AUDIO_CONTROLLER_GP2Y0E02B_REGISTER_MAP_H
//
// Created by bendstein on 12/22/2025.
//

#ifndef AUDIO_CONTROLLER_GP2Y0E02B_REGISTER_MAP_H
#define AUDIO_CONTROLLER_GP2Y0E02B_REGISTER_MAP_H

namespace gp2y0e02b
{
    enum struct register_map_tag : uint8_t
    {
        UNKNOWN,
        HOLD_BIT = 0x03,
        MAXIMUM_EMITTING_PULSE_WIDTH = 0x13,
        SPOT_SYMMETRY_THRESHOLD = 0x1C,
        SIGNAL_INTENSITY_THRESHOLD = 0x2F,
        MAXIMUM_SPOT_SIZE_THRESHOLD = 0x33,
        MINIMUM_SPOT_SIZE_THRESHOLD = 0x34,
        SHIFT_BIT = 0x35,
        MEDIAN_FILTER = 0x3F,
        SRAM_ACCESS = 0x4C,
        DISTANCE_11_THRU_4 = 0x5E,
        DISTANCE_3_THRU_0 = 0x5F,
        AE_15_THRU_8 = 0x64,
        AE_7_THRU_0 = 0x65,
        AG_7_THRU_0 = 0x67,
        COVER_COMPENSATION_5_THRU_0 = 0x8D,
        COVER_COMPENSATION_10_THRU_6 = 0x8E,
        COVER_COMPENSATION_ENABLE_BIT = 0x8F,
        READ_OUT_IMAGE_SENSOR_DATA = 0x90,
        SIGNAL_ACCUMULATION_NUMBER = 0xA8,
        SIGNAL_INTENSITY_ENABLE_BIT = 0xBC,
        MAXIMUM_SPOT_SIZE_ENABLE_BIT = 0xBD,
        MINIMUM_SPOT_SIZE_ENABLE_BIT = 0xBE,
        SPOT_SYMMETRY_ENABLE_BIT = 0xBF,
        EFUSE_TARGET_ADDRESS = 0xC8,
        EFUSE_BIT_NUMBER_AND_BANK_ASSIGN = 0xC9,
        EFUSE_PROGRAM_ENABLE_BIT = 0xCA,
        EFUSE_PROGRAM_DATA = 0xCD,
        ACTIVE_STAND_BY_STATE_CONTROL = 0xE8,
        CLOCK_SELECT = 0xEC,
        SOFTWARE_RESET = 0xEE,
        BANK_SELECT = 0xEF,
        RIGHT_EDGE_COORDINATE = 0xF8,
        LEFT_EDGE_COORDINATE = 0xF9,
        PEAK_COORDINATE = 0xFA
    };

    enum struct maximum_emitting_pulse_width : uint8_t
    {
        us_40 = 0x03,
        us_80 = 0x04,
        us_160 = 0x05,
        us_240 = 0x06,
        us_320 = 0x07,
    };

    enum struct cover_compensation_enable_bit : uint8_t
    {
        enable = 0x02,
        disable = 0x03
    };

    enum struct hold_bit : uint8_t
    {
        hold = 0x00,
        enable = 0x01,
    };

    enum struct enable_bit : uint8_t
    {
        enable = 0x00,
        disable = 0x01,
    };

    enum struct efuse_program_enable_bit : uint8_t
    {
        disable = 0x00,
        enable = 0x01,
    };

    enum struct shift_bit : uint8_t
    {
        cm_128 = 0x01,
        cm_64 = 0x02,
    };

    enum struct median_filter : uint8_t
    {
        med_7 = 0x00,
        med_5 = 0x01,
        med_9 = 0x02,
        med_1 = 0x03,
    };

    enum struct read_out_image_sensor_data : uint8_t
    {
        lvl_disable = 0x00,
        lvl_low = 0x01,
        lvl_mid = 0x11,
        lvl_high = 0x12,
    };

    enum struct signal_accumulation_number : uint8_t
    {
        times_1 = 0x00,
        times_5 = 0x01,
        times_30 = 0x02,
        times_10 = 0x03,
    };

    enum struct efuse_read_out : uint8_t
    {
        no_readout = 0x00,
        load_to_register = 0x01,
    };

    enum struct register_bank : uint8_t
    {
        bank_a = 0x01,
        bank_b = 0x02,
        bank_c = 0x03,
        bank_d = 0x04,
        bank_e = 0x05
    };

    enum struct active_stand_by_state : uint8_t
    {
        active = 0x00,
        standby = 0x01,
    };

    enum struct clock_select : uint8_t
    {
        clk_auto = 0x7F,
        clk_manual = 0xFF
    };

    enum struct software_reset : uint8_t
    {
        unset = 0x00,
        reset = 0x03
    };

    enum struct bank_select : uint8_t
    {
        select_bank_a = 0x00,
        select_bank_e = 0x03,
    };

    struct register_map_entry_hold_bit
    {
        hold_bit hold: 1;
    };

    struct register_map_entry_maximum_emitting_pulse_width
    {
        maximum_emitting_pulse_width pulse_width: 3;
    };

    struct register_map_entry_spot_symmetry_threshold
    {
        uint8_t threshold: 5;
    };

    struct register_map_entry_signal_intensity_threshold
    {
        uint8_t threshold: 7;
    };

    struct register_map_entry_maximum_spot_size_threshold
    {
        uint8_t threshold;
    };

    struct register_map_entry_minimum_spot_size_threshold
    {
        uint8_t threshold: 7;
    };

    struct register_map_entry_shift_bit
    {
        shift_bit shift: 3;
    };

    struct register_map_entry_median_filter
    {
        uint8_t _dummy: 5;
        median_filter filter: 3;
    };

    struct register_map_entry_sram_access
    {
        uint8_t _dummy: 4;
        uint8_t sram_access: 1;
    };

    struct register_map_entry_distance_11_thru_4
    {
        uint8_t distance_part;
    };

    struct register_map_entry_distance_3_thru_0
    {
        uint8_t distance_part: 4;
    };

    struct register_map_entry_ae_15_thru_8
    {
        uint8_t ae_part;
    };

    struct register_map_entry_ae_7_thru_0
    {
        uint8_t ae_part;
    };

    struct register_map_entry_ag_7_thru_0
    {
        uint8_t ag_part;
    };

    struct register_map_entry_cover_compensation_5_thru_0
    {
        uint8_t _dummy: 3;
        uint8_t cc_part: 5;
    };

    struct register_map_entry_cover_compensation_10_thru_6
    {
        uint8_t cc_part: 5;
    };

    struct register_map_entry_cover_compensation_enable_bit
    {
        cover_compensation_enable_bit enable: 2;
    };

    struct register_map_entry_read_out_image_sensor_data
    {
        read_out_image_sensor_data level: 5;
    };

    struct register_map_entry_signal_accumulation_number
    {
        signal_accumulation_number factor: 2;
    };

    struct register_map_entry_enable_bit
    {
        enable_bit enable: 1;
    };

    struct register_map_entry_efuse_target_address
    {
        uint8_t address: 6;
        efuse_read_out read_out: 1;
        enable_bit enable: 1;
    };

    struct register_map_entry_efuse_bit_number_and_bank_assign
    {
        register_bank bank_select: 4;
        uint8_t bit_number: 4;
    };

    struct register_map_entry_efuse_program_enable_bit
    {
        efuse_program_enable_bit enable: 1;
    };

    struct register_map_entry_efuse_program_data
    {
        uint8_t data;
    };

    struct register_map_entry_active_stand_by_state
    {
        active_stand_by_state state: 1;
    };

    struct register_map_entry_clock_select
    {
        clock_select clock;
    };

    struct register_map_entry_software_reset
    {
        uint8_t _dummy: 1;
        software_reset reset: 2;
    };

    struct register_map_entry_bank_select
    {
        bank_select bank: 2;
    };

    struct register_map_entry_coordinate
    {
        uint8_t coordinate;
    };

    union register_map_entry_value_union
    {
        uint8_t raw_value;
        register_map_entry_hold_bit hold_bit;
        register_map_entry_maximum_emitting_pulse_width maximum_emitting_pulse_width;
        register_map_entry_spot_symmetry_threshold spot_symmetry_threshold;
        register_map_entry_signal_intensity_threshold signal_intensity_threshold;
        register_map_entry_maximum_spot_size_threshold maximum_spot_size_threshold;
        register_map_entry_minimum_spot_size_threshold minimum_spot_size_threshold;
        register_map_entry_shift_bit shift_bit;
        register_map_entry_median_filter median_filter;
        register_map_entry_sram_access sram_access;
        register_map_entry_distance_11_thru_4 distance_11_thru_4;
        register_map_entry_distance_3_thru_0 distance_3_thru_0;
        register_map_entry_ae_15_thru_8 ae_15_thru_8;
        register_map_entry_ae_7_thru_0 ae_7_thru_0;
        register_map_entry_ag_7_thru_0 ag_7_thru_0;
        register_map_entry_cover_compensation_5_thru_0 cover_compensation_5_thru_0;
        register_map_entry_cover_compensation_10_thru_6 cover_compensation_10_thru_6;
        register_map_entry_cover_compensation_enable_bit cover_compensation_enable_bit;
        register_map_entry_read_out_image_sensor_data read_out_image_sensor_data;
        register_map_entry_signal_accumulation_number signal_accumulation_number;
        register_map_entry_enable_bit enable_bit_signal_intensity;
        register_map_entry_enable_bit enable_bit_minimum_spot_size;
        register_map_entry_enable_bit enable_bit_maximum_spot_size;
        register_map_entry_enable_bit enable_bit_spot_symmetry;
        register_map_entry_efuse_target_address efuse_target_address;
        register_map_entry_efuse_bit_number_and_bank_assign efuse_bit_number_and_bank_assign;
        register_map_entry_efuse_program_enable_bit efuse_program_enable_bit;
        register_map_entry_efuse_program_data efuse_program_data;
        register_map_entry_active_stand_by_state active_stand_by_state;
        register_map_entry_clock_select clock_select;
        register_map_entry_software_reset software_reset;
        register_map_entry_bank_select bank_select;
        register_map_entry_coordinate coordinate_right_edge;
        register_map_entry_coordinate coordinate_left_edge;
        register_map_entry_coordinate coordinate_peak;
    };

    struct register_map_entry
    {
        register_map_tag tag;
        register_map_entry_value_union data;

        [[nodiscard]]
        uint8_t get_register_address() const
        {
            return static_cast<uint8_t>(tag);
        }
    };
}

#endif //AUDIO_CONTROLLER_GP2Y0E02B_REGISTER_MAP_H
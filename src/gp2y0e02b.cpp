//
// Created by bendstein on 12/20/2025.
//
#include "app_common.h"
#include "gp2y0e02b.h"

#include <memory>
#include <driver/i2c.h>
#include <driver/i2c_master.h>

bool gp2y0e02b::distance_sensor::ping() const
{
    register_map_entry active_standby_entry = {
        .tag = register_map_tag::ACTIVE_STAND_BY_STATE_CONTROL,
        .data = {}
    };
    return try_read_from_register(&active_standby_entry);
}

bool gp2y0e02b::distance_sensor::try_update_distance_shift(shift_bit* distance_shift_out, shift_bit* prev_distance_shift_out)
{
    if (prev_distance_shift_out != nullptr) //Output prev value
        *prev_distance_shift_out = get_distance_shift();

    //Try to read shift bit, returning false if failed
    register_map_entry shift_entry = {
        .tag = register_map_tag::SHIFT_BIT,
        .data = {}
    };

    if (!try_read_from_register(&shift_entry))
        return false;

    state.distance_shift = shift_entry.data.shift_bit.shift; //Update cached value

    if (distance_shift_out != nullptr) //Output new value
        *distance_shift_out = shift_entry.data.shift_bit.shift;

    return true;
}

bool gp2y0e02b::distance_sensor::try_update_distance(uint8_t* distance_out, uint8_t* prev_distance_out)
{
    //TODO: Signal accumulation, Median filter both have an effect on the delay between setting the
    //register and reading it.

    if (prev_distance_out != nullptr) //Output prev value
        *prev_distance_out = get_distance();

    //Read distance parts
    register_map_entry register_entries[2] = {
        {
            .tag = register_map_tag::DISTANCE_11_THRU_4,
            .data = {}
        },
        {
            .tag = register_map_tag::DISTANCE_3_THRU_0,
            .data = {}
        }
    };

    if (!try_burst_read_from_register(register_entries, sizeof(register_entries) / sizeof(register_map_entry)))
        return false;

    //Combine values, as given in datasheet (16 * Distance[11:4] + Distance[3:0]) / 16 / 2^ShiftBit
    const auto distance_0 = register_entries[0].data.distance_11_thru_4.distance_part;
    const auto distance_1 = register_entries[1].data.distance_3_thru_0.distance_part;
    const auto shift_bit = get_distance_shift();

    /*
     * This is super overkill, but to justify the return type being an unsigned byte:
     *
     * Proof that (16 * Distance[11:4] + Distance[3:0]) / 16 / 2^ShiftBit will always fit in 8 bits:
     * Let:
     *   - distance_0 := *Distance[11:4]
     *   - distance_1 := *Distance[3:0]
     *   - shift_bit  := *ShiftBit
     * Given:
     *   - distance_0 bitfield uses all 8 bits (i.e. distance_0 is [0000_0000b, 1111_1111b])
     *   - distance_1 bitfield uses only 4 bits (i.e. distance_1 is [0000b, 1111b])
     *   - shift_bit bitfield uses only 3 bits (i.e. shift_bit is [0, 8])
     *     - per datasheet should only ever be 1 or 2
     * Then:
     *   - 16 * distance_0 = distance_0 << 4
     *       = [0000_0000b, 1111_1111b] << 4
     *       = [0000_0000_0000b, 1111_1111_0000b]
     *   - 16 * distance_0 + distance_1
     *       = [0000_0000_0000b, 1111_1111_0000b] + distance_1
     *       = [0000_0000_0000b, 1111_1111_0000b] + [0000b, 1111b]
     *       = [0000_0000_0000b, 1111_1111_1111b]
     *   - (16 * distance_0 + distance_1) / 16
     *       = [0000_0000_0000b, 1111_1111_1111b] / 16
     *       = [0000_0000_0000b, 1111_1111_1111b] >> 4
     *       = [0000_0000b, 1111_1111b]
     *   - (16 * distance_0 + distance_1) / 16 / 2^shift_bit
     *       = [0000_0000b, 1111_1111b] / 2^shift_bit
     *       = [0000_0000b, 1111_1111b] / 1 << shift_bit
     *       = [0000_0000b, 1111_1111b] >> shift_bit
     *       = [0000_0000b, 1111_1111b] >> [0, 8]
     *       = [0000_0000b, 1111_1111b]
     *   - Technically, then, can be anywhere from [0000_0000b, 1111_1111b],
     *     but since shift_bit should only be 1 or 2:
     * Therefore, range of values is:
     *   - [0000_0000b, 0111_1111b] if shift bit = 1
     *   - [0000_0000b, 0011_1111b] if shift bit = 2
     * Both of which can be represented using a uint8_t
     */
    const auto distance = static_cast<uint8_t>((((distance_0 << 4) + distance_1) >> 4) >> static_cast<uint8_t>(shift_bit));
    state.distance = distance; //Update cached value

    if (distance_out != nullptr) //Output new value
        *distance_out = distance;

    return true;
}

bool gp2y0e02b::distance_sensor::try_read_from_register(register_map_entry* entry) const
{
    const uint8_t target_register = entry->get_register_address();
    uint8_t buffer_read = 0;

    //Select register
    const auto result_select_register = i2c_master_transmit(
        handle,
        &target_register, 1,
        timeout_ms
    );

    if (result_select_register != ESP_OK)
        return false;

    //Data hold/setup time
    vTaskDelay(1 / portTICK_PERIOD_US);

    //Read from selected register
    const auto result_read_register = i2c_master_receive(
        handle,
        &buffer_read, 1,
        timeout_ms
    );

    if (result_read_register != ESP_OK)
        return false;

    entry->data.raw_value = buffer_read;

    return true;
}

bool gp2y0e02b::distance_sensor::try_burst_read_from_register(register_map_entry entries[], const size_t read_len) const
{
    assert(read_len > 0);

    //If read length is 1, call standard read from register
    if (read_len == 1)
    {
        return try_read_from_register(&entries[0]);
    }

    //Registers MUST be adjacent for burst read.
    auto prev_address = 0;
    for (size_t i = 0; i < read_len; i++)
    {
        const auto current_addr = entries[i].get_register_address();

        if (i > 0)
            assert(current_addr == prev_address + 1);

        prev_address = current_addr;
    }

    const uint8_t target_register = entries[0].get_register_address();

    //Select register
    const auto result_select_register = i2c_master_transmit(
        handle,
        &target_register, 1,
        timeout_ms
    );

    if (result_select_register != ESP_OK)
        return false;

    //Data hold/setup time
    vTaskDelay(1 / portTICK_PERIOD_US);

    //alloc read buffer that will be freed at end of scope
    const auto buffer_read = std::make_unique<uint8_t[]>(read_len);

    //Read starting from selected register
    const auto result_read_register = i2c_master_receive(
        handle,
        buffer_read.get(), read_len,
        timeout_ms
    );

    if (result_read_register != ESP_OK)
        return false;

    //Populate results
    for (size_t i = 0; i < read_len; i++)
    {
        entries[i].data.raw_value = buffer_read[i];
    }

    return true;
}

bool gp2y0e02b::distance_sensor::try_write_to_register(const register_map_entry* entry) const
{
    const uint8_t target_register = entry->get_register_address();

    //Select register
    const auto result_select_register = i2c_master_transmit(
        handle,
        &target_register, 1,
        timeout_ms
    );

    if (result_select_register != ESP_OK)
        return false;

    //Data hold/setup time
    vTaskDelay(1 / portTICK_PERIOD_US);

    //Write to selected register
    const auto result_read_register = i2c_master_transmit(
        handle,
        &entry->data.raw_value, 1,
        timeout_ms
    );

    return  result_read_register == ESP_OK;
}
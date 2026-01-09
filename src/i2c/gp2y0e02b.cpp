//
// Created by bendstein on 12/20/2025.
//
#include "app_common.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

#include <memory>
#include <driver/i2c.h>
#include <driver/i2c_master.h>

bool gp2y0e02b::distance_sensor::ping() const
{
    VERBOSE(get_log_key(), "Ping");

    register_map_entry active_standby_entry = {
        .tag = register_map_tag::ACTIVE_STAND_BY_STATE_CONTROL,
        .data = {}
    };

    return try_read_from_register(&active_standby_entry);
}

bool gp2y0e02b::distance_sensor::try_update_distance_shift(shift_bit* distance_shift_out, shift_bit* prev_distance_shift_out)
{
    VERBOSE(get_log_key(), "Update distance shift");

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
    VERBOSE(get_log_key(), "Update distance");

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

    VERBOSE(get_log_key(), std::format("New distance: {}cm", distance));

    return true;
}

bool gp2y0e02b::distance_sensor::try_apply_distance_shift(const shift_bit new_shift_bit, shift_bit* prev_distance_shift_out)
{
    VERBOSE(get_log_key(), "Apply distance shift");

    if (prev_distance_shift_out != nullptr) //Output prev value
        *prev_distance_shift_out = get_distance_shift();

    const register_map_entry shift_entry = {
        .tag = register_map_tag::SHIFT_BIT,
        .data = {
            .shift_bit = {
                .shift = new_shift_bit
            }
        }
    };

    if (!try_write_to_register(&shift_entry))
        return false;

    state.distance_shift = new_shift_bit; //Update cached value

    return true;
}

bool gp2y0e02b::distance_sensor::try_soft_reset()
{
    VERBOSE(get_log_key(), "Soft reset");

    //Set clock to manual
    constexpr register_map_entry clock_manual_entry = {
        .tag = register_map_tag::CLOCK_SELECT,
        .data = {
            .clock_select = {
                .clock = clock_select::clk_manual
            }
        }
    };

    if (!try_write_to_register(&clock_manual_entry))
        return false;

    //After this point, we want to set clock back to automatic,
    //even if setting the software reset register fails
    auto is_success = true;

    //Write to reset register to start reset
    constexpr register_map_entry reset_entry = {
        .tag = register_map_tag::SOFTWARE_RESET,
        .data = {
            .software_reset = {
                ._dummy = 0,
                .reset = software_reset::reset
            }
        }
    };

    if (!try_write_to_register(&reset_entry))
        is_success = false;

    //Set clock to automatic
    constexpr register_map_entry clock_auto_entry = {
        .tag = register_map_tag::CLOCK_SELECT,
        .data = {
            .clock_select = {
                .clock = clock_select::clk_auto
            }
        }
    };

    if (!try_write_to_register(&clock_auto_entry))
        return false;

    if (!is_success)
        return false;

    //If successful, set cached state back to default, as it
    //should no longer be valid
    state.reset();
    return true;
}

bool gp2y0e02b::distance_sensor::try_select_register(register_map_tag tag) const
{
    try
    {
        VERBOSE(get_log_key(), std::format("Select register 0x{:02X}", static_cast<uint8_t>(tag)));

        const auto target_register = static_cast<uint8_t>(tag);
        const auto result_select_register = i2c_master_transmit(
            handle,
            &target_register, 1,
            timeout_ms
        );

        VERBOSE(get_log_key(), std::format("Select register 0x{:02X} result: 0x{:04X}",
            static_cast<uint8_t>(tag),
            result_select_register));

        if (result_select_register != ESP_OK)
        {
            loge(get_log_key(), std::format("Failed to select register {}. [0x{:04X}] {}",
                static_cast<uint8_t>(tag),
                result_select_register,
                esp_err_to_name(result_select_register)));
        }

        return result_select_register == ESP_OK;
    }
    catch (const std::exception& e)
    {
        loge(get_log_key(), std::format("An exception occurred while selecting register {}: {}",
            static_cast<uint8_t>(tag),
            e.what()));

        return false;
    }
}

bool gp2y0e02b::distance_sensor::try_read_from_register(register_map_entry* entry) const
{
    try
    {
        uint8_t buffer_read = 0;

        VERBOSE(get_log_key(), std::format("Read from register 0x{:02X}", entry->get_register_address()));

        //Select register
        if (!try_select_register(entry->tag))
            return false;

        //Data hold/setup time
        vTaskDelay(1 / portTICK_PERIOD_US);

        VERBOSE(get_log_key(), "Send read request.");

        //Read from selected register
        const auto result_read_register = i2c_master_receive(
            handle,
            &buffer_read, 1,
            timeout_ms
        );

        VERBOSE(get_log_key(), std::format("Read result: 0x{:04X}", result_read_register));

        if (result_read_register != ESP_OK)
        {
            loge(get_log_key(), std::format("Failed to read from register 0x{:02X}. [0x{:04X}] {}",
                entry->get_register_address(),
                result_read_register,
                esp_err_to_name(result_read_register)));

            return false;
        }

        entry->data.raw_value = buffer_read;

        VERBOSE(get_log_key(), std::format("Read value: 0x{:02X}", entry->data.raw_value));

        return true;
    }
    catch (std::exception& e)
    {
        loge(get_log_key(), std::format("An exception occurred while reading from register 0x{:02X}: {}",
            entry->get_register_address(),
            e.what()));

        return false;
    }
}

bool gp2y0e02b::distance_sensor::try_burst_read_from_register(register_map_entry entries[], const size_t read_len) const
{
    try
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

        VERBOSE(get_log_key(), std::format("Burst read {} registers, starting at address 0x{:02X}.",
            read_len,
            entries[0].get_register_address()));

        //Select register
        if (!try_select_register(entries[0].tag))
            return false;

        //Data hold/setup time
        vTaskDelay(1 / portTICK_PERIOD_US);

        //alloc read buffer that will be freed at end of scope
        const auto buffer_read = std::make_unique<uint8_t[]>(read_len);

        VERBOSE(get_log_key(), "Send burst read request.");

        //Read starting from selected register
        const auto result_read_register = i2c_master_receive(
            handle,
            buffer_read.get(), read_len,
            timeout_ms
        );

        VERBOSE(get_log_key(), std::format("Burst read result: 0x{:04X}", result_read_register));

        if (result_read_register != ESP_OK)
        {
            loge(get_log_key(), std::format("Failed to burst read {} items starting from register 0x{:02X}. [0x{:04X}] {}",
                read_len,
                entries[0].get_register_address(),
                result_read_register,
                esp_err_to_name(result_read_register)));

            return false;
        }

        //Populate results
        for (size_t i = 0; i < read_len; i++)
        {
            VERBOSE(get_log_key(), std::format("Burst read result {}: 0x{:02X}", i, buffer_read[i]));
            entries[i].data.raw_value = buffer_read[i];
        }

        return true;
    }
    catch (std::exception& e)
    {
        loge(get_log_key(), std::format("An exception occurred while burst reading {} items starting from register 0x{:02X}: {}",
            read_len,
            read_len < 1? -1 : entries[0].get_register_address(),
            e.what()));

        return false;
    }
}

bool gp2y0e02b::distance_sensor::try_write_to_register(const register_map_entry* entry) const
{
    try
    {
        VERBOSE(get_log_key(), std::format("Write to register 0x{:02X}", entry->get_register_address()));

        //Select register
        if (!try_select_register(entry->tag))
            return false;

        //Data hold/setup time
        vTaskDelay(1 / portTICK_PERIOD_US);

        VERBOSE(get_log_key(), std::format("Send write request with data: 0x{:02X}", entry->data.raw_value));

        //Write to selected register
        const auto result_write_register = i2c_master_transmit(
            handle,
            &entry->data.raw_value, 1,
            timeout_ms
        );

        if (result_write_register != ESP_OK)
        {
            loge(get_log_key(), std::format("Failed to write value 0x{:02X} to register 0x{:02X}. [0x{:04X}] {}",
                entry->data.raw_value,
                entry->get_register_address(),
                result_write_register,
                esp_err_to_name(result_write_register)));
        }

        return result_write_register == ESP_OK;
    }
    catch (std::exception& e)
    {
        loge(get_log_key(), std::format("An exception occurred while writing value 0x{:02X} to register 0x{:02X}: {}",
            entry->data.raw_value,
            entry->get_register_address(),
            e.what()));

        return false;
    }
}
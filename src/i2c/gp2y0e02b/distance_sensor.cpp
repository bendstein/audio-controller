//
// Created by bendstein on 12/20/2025.
//
#include <memory>

#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"

#include "app_common.h"
#include "i2c/i2c.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

gp2y0e02b::distance_sensor::distance_sensor(i2c_master_dev_handle_t device_handle, const uint8_t address, const size_t index)
    : handle(device_handle), address(address), index(index)
{
    state.reset();
    log_key = make_log_key();
}

gp2y0e02b::distance_sensor::distance_sensor(i2c_master_dev_handle_t device_handle, const uint8_t address, const size_t index, const int32_t timeout)
    : distance_sensor(device_handle, address, index)
{
    timeout_ms = timeout;
}

gp2y0e02b::distance_sensor::~distance_sensor()
{
    //Remove device from bus on destroy
    if (handle != nullptr)
    {
        try
        {
            FLOGI("{} Removing device 0x{:08X} from bus.",
                 log_key,
                 reinterpret_cast<uintptr_t>(handle));

            if (const auto rm_device_result = i2c_master_bus_rm_device(handle);
                rm_device_result != ESP_OK)
            {
                FLOGE("{} Failed to remove device 0x{:08X} from bus. [0x{:04X}] {}" , log_key,
                    reinterpret_cast<uintptr_t>(handle), rm_device_result, esp_err_to_name(rm_device_result));
            }
        }
        catch (std::exception& e)
        {
            FLOGE("{} An exception occurred while removing device 0x{:08X} from bus: {}" , log_key,
                reinterpret_cast<uintptr_t>(handle), e.what());
        }
    }
}

std::string gp2y0e02b::distance_sensor::make_log_key() const
{
    return std::format("[distance sensor 0x{:02X}; 0x{:08X}]",
        address,
        reinterpret_cast<uintptr_t>(handle));
}
bool gp2y0e02b::distance_sensor::ping() const
{
    FLOGI("[{}] Ping.", get_log_key());

    register_map_entry active_standby_entry = {
        .tag = register_map_tag::ACTIVE_STAND_BY_STATE_CONTROL,
        .data = {}
    };

    return try_read_from_register(&active_standby_entry);
}

bool gp2y0e02b::distance_sensor::try_update_distance_shift(shift_bit* distance_shift_out, shift_bit* prev_distance_shift_out)
{
    FLOGI("[{}] Update distance shift.", get_log_key());

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
    FVERBOSE("[{}] Update distance.", get_log_key());

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

    FVERBOSE("[{}] New distance: {}cm.", get_log_key(), distance);

    return true;
}

bool gp2y0e02b::distance_sensor::try_apply_distance_shift(const shift_bit new_shift_bit, shift_bit* prev_distance_shift_out)
{
    FLOGI("[{}] Apply distance shift.", get_log_key());

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
    FLOGW("[{}] Soft reset.", get_log_key());

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
        assert(handle != nullptr);
        FVERBOSE("[{}] Select register 0x{:02X}.", get_log_key(), static_cast<uint8_t>(tag));

        const auto target_register = static_cast<uint8_t>(tag);
        const auto result_select_register = i2c_master_transmit(
            handle,
            &target_register, 1,
            timeout_ms
        );

        FVERBOSE("[{}] Select register 0x{:02X} result: 0x{:04X}", get_log_key(), static_cast<uint8_t>(tag), result_select_register);

        if (result_select_register != ESP_OK)
        {
            FLOGE("[{}] Failed to select register {}. [0x{:04X}] {}" , get_log_key(), static_cast<uint8_t>(
                tag), result_select_register, esp_err_to_name(result_select_register));
        }

        return result_select_register == ESP_OK;
    }
    catch (const std::exception& e)
    {
        FLOGE("[{}] An exception occurred while selecting register {}: {}" , get_log_key(), static_cast<
            uint8_t>(tag), e.what());

        return false;
    }
}

bool gp2y0e02b::distance_sensor::try_read_from_register(register_map_entry* entry) const
{
    try
    {
        assert(handle != nullptr);
        uint8_t buffer_read = 0;

        FVERBOSE("[{}] Read from register 0x{:02X}.", get_log_key(), entry->get_register_address());

        //Select register
        if (!try_select_register(entry->tag))
            return false;

        //Data hold/setup time
        vTaskDelay(1 / portTICK_PERIOD_MS);

        FVERBOSE("[{}] Send read request.", get_log_key());

        //Read from selected register
        const auto result_read_register = i2c_master_receive(
            handle,
            &buffer_read, 1,
            timeout_ms
        );

        FVERBOSE("[{}] Read result: 0x{:04X}.", get_log_key(), result_read_register);

        if (result_read_register != ESP_OK)
        {
            FLOGE("[{}] Failed to read from register 0x{:02X}. [0x{:04X}] {}" , get_log_key(), entry->
                get_register_address(), result_read_register, esp_err_to_name(result_read_register));

            return false;
        }

        entry->data.raw_value = buffer_read;

        FVERBOSE("[{}] Read value 0x{:02X}.", get_log_key(), entry->data.raw_value);

        return true;
    }
    catch (std::exception& e)
    {
        FLOGE("[{}] An exception occurred while reading from register 0x{:02X}: {}" , get_log_key(), entry->
            get_register_address(), e.what());

        return false;
    }
}

bool gp2y0e02b::distance_sensor::try_burst_read_from_register(register_map_entry entries[], const size_t read_len) const
{
    try
    {
        assert(handle != nullptr);
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

        FVERBOSE("[{}] Burst read {} registers, starting at address 0x{:02X}.", get_log_key(), read_len,
            entries[0].get_register_address());

        //Select register
        if (!try_select_register(entries[0].tag))
            return false;

        //Data hold/setup time
        vTaskDelay(1 / portTICK_PERIOD_MS);

        //alloc read buffer that will be freed at end of scope
        const auto buffer_read = std::make_unique<uint8_t[]>(read_len);

        FVERBOSE("[{}] Send burst read request.", get_log_key());

        //Read starting from selected register
        const auto result_read_register = i2c_master_receive(
            handle,
            buffer_read.get(), read_len,
            timeout_ms
        );

        FVERBOSE("[{}] Burst read result: 0x{:04X}.", get_log_key(), result_read_register);

        if (result_read_register != ESP_OK)
        {
            FLOGE("[{}] Failed to burst read {} items starting from register 0x{:02X}. [0x{:04X}] {}" ,
                get_log_key(), read_len, entries[0].get_register_address(), result_read_register, esp_err_to_name(
                    result_read_register));

            return false;
        }

        //Populate results
        for (size_t i = 0; i < read_len; i++)
        {
            FVERBOSE("[{}] Burst read {}: 0x{:02X}.", get_log_key(), i, buffer_read[i]);
            entries[i].data.raw_value = buffer_read[i];
        }

        return true;
    }
    catch (std::exception& e)
    {
        FLOGE("[{}] An exception occurred while burst reading {} items starting from register 0x{:02X}: {}" ,
            get_log_key(), read_len, read_len < 1? -1 : entries[0].get_register_address(), e.what());

        return false;
    }
}

bool gp2y0e02b::distance_sensor::try_write_to_register(const register_map_entry* entry) const
{
    try
    {
        assert(handle != nullptr);

        FVERBOSE("[{}] Write to register 0x{:02X}.", get_log_key(), entry->get_register_address());

        //Select register
        if (!try_select_register(entry->tag))
            return false;

        //Data hold/setup time
        vTaskDelay(1 / portTICK_PERIOD_MS);

        FVERBOSE("[{}] Send write request with data: 0x{:02X}", get_log_key(), entry->data.raw_value);

        //Write to selected register
        const auto result_write_register = i2c_master_transmit(
            handle,
            &entry->data.raw_value, 1,
            timeout_ms
        );

        if (result_write_register != ESP_OK)
        {
            FLOGE("[{}] Failed to write value 0x{:02X} to register 0x{:02X}. [0x{:04X}] {}" , get_log_key(),
                entry->data.raw_value, entry->get_register_address(), result_write_register, esp_err_to_name(
                    result_write_register));
        }

        return result_write_register == ESP_OK;
    }
    catch (std::exception& e)
    {
        FLOGE("[{}] An exception occurred while writing value 0x{:02X} to register 0x{:02X}: {}" ,
            get_log_key(), entry->data.raw_value, entry->get_register_address(), e.what());

        return false;
    }
}

std::unique_ptr<gp2y0e02b::distance_sensor> gp2y0e02b::distance_sensor::try_create_on_bus(i2c_master_bus_handle_t bus, const uint8_t addr, const size_t index, const int32_t timeout_ms)
{
    FLOGI("[distance sensor 0x{:02X}] Creating device on bus 0x{:08X}.", addr, reinterpret_cast<uintptr_t>(bus));

    const i2c_device_config_t device_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = static_cast<uint8_t>(addr >> 1),
        .scl_speed_hz = I2C_STANDARD_HZ,
        .scl_wait_us = I2C_SCL_WAIT_US_DFT,
        .flags = {
            .disable_ack_check = false
        }
    };

    i2c_master_dev_handle_t handle;

    if (const auto add_to_bus_result = i2c_master_bus_add_device(bus, &device_cfg, &handle); add_to_bus_result != ESP_OK)
    {
        FLOGE("[distance sensor 0x{:02X}] Failed to create device on bus 0x{:08X}. [0x{:04X}] {}" , addr,
            reinterpret_cast<uintptr_t>(bus), add_to_bus_result, esp_err_to_name(add_to_bus_result));

        return nullptr;
    }

    return std::make_unique<distance_sensor>(handle, addr, index, timeout_ms);
}

void gp2y0e02b::distance_sensor::permanently_apply_new_i2c_address(i2c_master_bus_handle_t bus, uint8_t addr_new)
{
    //Per docs, only top 4 bits are available for addressing.
    //Bottom four bits must be 0 for this purpose.
    assert((addr_new & 0x0F) == 0);
    assert(bus != nullptr);

    const auto maybe_sensor = try_create_on_bus(bus, I2C_ADDR_DFT, 0, -1);

    if (maybe_sensor == nullptr)
        throw std::runtime_error("Failed to create sensor");

    const auto& sensor = *maybe_sensor;

    FLOGI("Programming sensor e-fuses to use address 0x{:02X}.", addr_new);

    //The steps below are taken from the datasheet, table 20.

    vTaskDelay(1 / portTICK_PERIOD_MS); //Wait 1ms for Vcc

    //Stage 1

    //Set clock to manual
    LOGI("Step 1: set clock to manual");
    constexpr register_map_entry clock_select_entry = {
        .tag = register_map_tag::CLOCK_SELECT,
        .data = {
            .clock_select = {
                .clock = clock_select::clk_manual
            }
        }
    };

    if (!sensor.try_write_to_register(&clock_select_entry))
        throw std::runtime_error("Failed to write clock select");

    //Enable Vpp
    LOGI("Step 1.1: Set Vpp to high");
    gpio_set_level(PIN_VPP_ENABLE, HIGH);
    vTaskDelay(1 / portTICK_PERIOD_MS); //Wait 1ms for Vpp

    //Stage 2
    //Select e-fuse target (haven't selected bank yet, but
    //this is to set target to the very start of bank e,
    //which is where the i2c address is set)
    LOGI("Step 2: Select target address for update in e-fuse bank.");
    constexpr register_map_entry efuse_target_entry = {
        .tag = register_map_tag::EFUSE_TARGET_ADDRESS,
        .data = {
            .efuse_target_address = {
                .address = 0x00,
                .read_out = efuse_read_out::no_readout,
                .enable = enable_bit::enable
            }
        }
    };

    if (!sensor.try_write_to_register(&efuse_target_entry))
        throw std::runtime_error("Failed to select target e-fuse");

    //Stage 3
    //Specify the bank to write in, and the bit number.
    //i2c address is in bank e.
    //As far as I can tell, we use 0x04 because that's the
    //length of the i2c address in memory, with lsb at 0x00
    //set in the previous write.
    //The process described in the datasheet is unclear, so
    //this is just my best guess.
    LOGI("Step 3: Select e-fuse bank and bit number for update.");
    constexpr register_map_entry bank_target_entry = {
        .tag = register_map_tag::EFUSE_BIT_NUMBER_AND_BANK_ASSIGN,
        .data = {
            .efuse_bit_number_and_bank_assign = {
                .bank_select = register_bank::bank_e,
                .bit_number = 0x04,
            }
        }
    };

    if (!sensor.try_write_to_register(&bank_target_entry))
        throw std::runtime_error("Failed to select target e-fuse bank");

    //Stage 4

    //Set new address in efuse program register
    LOGI("Step 4: Set new i2c address in efuse program register.");
    const register_map_entry set_new_address_entry = {
        .tag = register_map_tag::EFUSE_PROGRAM_DATA,
        .data = {
            .efuse_program_data = {
                .data = static_cast<uint8_t>(addr_new >> 4) //>> 4 bc cannot program bottom 4 bits
            }
        }
    };

    if (!sensor.try_write_to_register(&set_new_address_entry))
        throw std::runtime_error("Failed to set new address in e-fuse program register");

    //Stage 5

    //Enable programming (start commit)
    LOGI("Step 5, 6: Enable e-fuse program, wait 500us, and then disable e-fuse program, to apply changes.");
    constexpr register_map_entry start_program_entry = {
        .tag = register_map_tag::EFUSE_PROGRAM_ENABLE_BIT,
        .data = {
            .efuse_program_enable_bit = {
                .enable = efuse_program_enable_bit::enable
            }
        }
    };

    if (!sensor.try_write_to_register(&start_program_entry))
        throw std::runtime_error("Failed to enable/start e-fuse program");

    vTaskDelay(1 / portTICK_PERIOD_MS); //Delay while writing program

    //Stage 6

    //Disable programming (end commit)
    constexpr register_map_entry end_program_entry = {
        .tag = register_map_tag::EFUSE_PROGRAM_ENABLE_BIT,
        .data = {
            .efuse_program_enable_bit = {
                .enable = efuse_program_enable_bit::disable
            }
        }
    };

    if (!sensor.try_write_to_register(&end_program_entry))
        throw std::runtime_error("Failed to disable/end e-fuse program");

    LOGI("Step 6.1: Finished applying e-fuse program. Bringing Vpp back to low.");

    //Disable Vpp
    gpio_set_level(PIN_VPP_ENABLE, LOW);
    vTaskDelay(1 / portTICK_PERIOD_MS); //Wait 1ms for Vpp

    //Stage 7

    //Change bank to control register
    LOGI("Step 7: Selecting control register/bank a.");
    constexpr register_map_entry select_control_register_entry = {
        .tag = register_map_tag::BANK_SELECT,
        .data = {
            .bank_select = {
                .bank = bank_select::select_bank_a
            }
        }
    };

    if (!sensor.try_write_to_register(&select_control_register_entry))
        throw std::runtime_error("Failed to select control register");

    //Update register values from e-fuses
    LOGI("Step 7.1: enable loading e-fuse data into register.");
    constexpr register_map_entry enable_update_register_entry = {
        .tag = register_map_tag::EFUSE_TARGET_ADDRESS,
        .data = {
            .efuse_target_address = {
                .address = 0x00,
                .read_out = efuse_read_out::load_to_register,
                .enable = enable_bit::enable
            }
        }
    };

    if (!sensor.try_write_to_register(&enable_update_register_entry))
        throw std::runtime_error("Failed to update register data from e-fuses");

    //Stop updating register values from e-fuses
    LOGI("Step 7.2: disable loading e-fuse data into register.");
    constexpr register_map_entry disable_update_register_entry = {
        .tag = register_map_tag::EFUSE_TARGET_ADDRESS,
        .data = {
            .efuse_target_address = {
                .address = 0x00,
                .read_out = efuse_read_out::no_readout,
                .enable = enable_bit::enable
            }
        }
    };

    if (!sensor.try_write_to_register(&disable_update_register_entry))
        throw std::runtime_error("Failed to stop updating register data from e-fuses");

    //Stage 8

    //Software reset
    LOGI("Step 8: Restarting sensor.");

    constexpr register_map_entry reset_software_entry = {
        .tag = register_map_tag::SOFTWARE_RESET,
        .data = {
            .software_reset = {
                ._dummy = 0,
                .reset = software_reset::reset
            }
        }
    };

    if (!sensor.try_write_to_register(&reset_software_entry))
        throw std::runtime_error("Failed to restart sensor");

    //Stages 9, 10 are validation and error-handling.
}
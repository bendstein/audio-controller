//
// Created by bendstein on 12/20/2025.
//
#include "app_common.h"
#include "gp2y0e02b.h"

#include <memory>
#include <driver/i2c.h>
#include <driver/i2c_master.h>

/**
 *
 * @param bus
 * @param addr_new
 * @warning This is a destructive action, as it permanently burns e-fuses.
 */
void gp2y0e02b::set_i2c_addr(i2c_master_bus_handle_t bus, uint8_t addr_new)
{
    //Per docs, only top 4 bits are available for addressing.
    //Bottom four bits must be 0 for this purpose.
    assert((addr_new & 0x0F) == 0);
    assert(bus != nullptr);

    logi("gp2y0e02b", std::format("Programming sensor efuses to use address 0x{:02X}.", addr_new));

    //Connect to device
    const auto device = i2c_init_device(bus, GP2Y0E02B_I2C_ADDR_DFT, i2c_device_type::GP2Y0E02B);

    return;

    vTaskDelay(1 / portTICK_PERIOD_MS); //Wait 1ms for Vcc

    //Stage 1

    //Clock Select (0xEC) := Manual (0xFF)
    constexpr uint8_t buffer_clock_sel[] = { 0xEC, 0xFF };
    ESP_ERROR_CHECK(i2c_master_transmit(device.handle, buffer_clock_sel, sizeof(buffer_clock_sel), -1));

    //Enable Vpp
    gpio_set_level(GP2Y0E02B_PIN_ENABLE_VPP, HIGH);
    vTaskDelay(1 / portTICK_PERIOD_MS); //Wait 1ms for Vpp

    //Stage 2

    //Target addr (0xC8) := 0x00 (lsb of peripheral id in bank e)
    constexpr uint8_t buffer_target_lsb_sel[] = { 0xC8, 0x00 };
    ESP_ERROR_CHECK(i2c_master_transmit(device.handle, buffer_target_lsb_sel, sizeof(buffer_target_lsb_sel), -1));

    //Stage 3

    //Target bit/bank (0xC9) := 0x45 (Specify the bits/bank to set values for)
    constexpr uint8_t buffer_target_bits_sel[] = { 0xC9, 0x45 };
    ESP_ERROR_CHECK(i2c_master_transmit(device.handle, buffer_target_bits_sel, sizeof(buffer_target_bits_sel), -1));

    //Stage 4

    //Data (0xCD) := addr_new (Set data)
    //>> 4 bc cannot program bottom 4 bits (handled by assert at top)
    const uint8_t buffer_set_data[] = { 0xCA, static_cast<uint8_t>(addr_new >> 4) };
    ESP_ERROR_CHECK(i2c_master_transmit(device.handle, buffer_set_data, sizeof(buffer_set_data), -1));

    //Stage 5

    //Program Enable (0xCA) := 0x01 (Start commit)
    constexpr uint8_t buffer_write_program_start[] = { 0xCA, 0x01 };
    ESP_ERROR_CHECK(i2c_master_transmit(device.handle, buffer_write_program_start, sizeof(buffer_write_program_start), -1));

    vTaskDelay(500 / portTICK_PERIOD_US); //Delay 500 microseconds while writing program

    //Stage 6

    //Program Enable (0xCA) := 0x00 (End commit)
    constexpr uint8_t buffer_write_program_end[] = { 0xCA, 0x00 };
    ESP_ERROR_CHECK(i2c_master_transmit(device.handle, buffer_write_program_end, sizeof(buffer_write_program_end), -1));

    //Disable Vpp
    gpio_set_level(GP2Y0E02B_PIN_ENABLE_VPP, LOW);
    vTaskDelay(1 / portTICK_PERIOD_MS); //Wait 1ms for Vpp

    //Stage 7

    constexpr uint8_t buffer_stage_7_0[] = { 0xEF, 0x00 }; //Bank Select (0xEF) := 0x00 (Digital control register)
    constexpr uint8_t buffer_stage_7_1[] = { 0xC8, 0x40 }; //E-Fuse read out (0xC8:6) := 0x40 (Load E-Fuse data to register)
    constexpr uint8_t buffer_stage_7_2[] = { 0xC8, 0x00 }; //E-Fuse read out (0xC8:6) := 0x00 (Unset prev)

    ESP_ERROR_CHECK(i2c_master_transmit(device.handle, buffer_stage_7_0, sizeof(buffer_stage_7_0), -1));
    ESP_ERROR_CHECK(i2c_master_transmit(device.handle, buffer_stage_7_1, sizeof(buffer_stage_7_1), -1));
    ESP_ERROR_CHECK(i2c_master_transmit(device.handle, buffer_stage_7_2, sizeof(buffer_stage_7_2), -1));

    //Stage 8

    //Software reset (0xEE) := 0x06 (Reset)
    constexpr uint8_t buffer_stage_8[] = { 0xEE, 0x06 };
    ESP_ERROR_CHECK(i2c_master_transmit(device.handle, buffer_stage_8, sizeof(buffer_stage_8), -1));

    // //Stage 9
    //
    // constexpr uint8_t buffer_stage_9_0[] = { 0xEF, 0x00 }; //Bank Select (0xEF) := 0x00 (Digital control register)
    // constexpr uint8_t buffer_stage_9_1[] = { 0xEC, 0xFF }; //Clock Select (0xEC) := Manual (0xFF)
    // constexpr uint8_t buffer_stage_9_2[] = { 0xEF, 0x03 }; //Bank select (0xEF) := 0x03 (E-fuse register)
    //
    // ESP_ERROR_CHECK(i2c_master_transmit(device.handle, buffer_stage_9_0, sizeof(buffer_stage_9_0), -1));
    // ESP_ERROR_CHECK(i2c_master_transmit(device.handle, buffer_stage_9_1, sizeof(buffer_stage_9_1), -1));
    // ESP_ERROR_CHECK(i2c_master_transmit(device.handle, buffer_stage_9_2, sizeof(buffer_stage_9_2), -1));
    //
    //
}

bool gp2y0e02b::distance_sensor::ping() const { return read_from_register(register_map_tag::ACTIVE_STAND_BY_STATE_CONTROL).has_value(); }

bool gp2y0e02b::distance_sensor::update_distance_shift(shift_bit* distance_shift_out, shift_bit* prev_distance_shift_out)
{
    if (prev_distance_shift_out != nullptr) //Output prev value
        *prev_distance_shift_out = get_distance_shift();

    //Read shift bit
    const auto maybe_shift_bit_entry = read_from_register(register_map_tag::SHIFT_BIT);

    if (!maybe_shift_bit_entry.has_value())
        return false;

    const auto shift_bit = maybe_shift_bit_entry.value().data.shift_bit.shift;
    state.distance_shift = shift_bit; //Update cached value

    if (distance_shift_out != nullptr) //Output new value
        *distance_shift_out = shift_bit;

    return true;
}

bool gp2y0e02b::distance_sensor::update_distance(uint8_t* distance_out, uint8_t* prev_distance_out)
{
    //TODO: Signal accumulation, Median filter both have an effect on the delay between setting the
    //register and reading it.

    if (prev_distance_out != nullptr) //Output prev value
        *prev_distance_out = get_distance();

    //Read distance parts
    register_map_entry register_entries[2];

    constexpr register_map_tag registers_to_read[] = {
        register_map_tag::DISTANCE_11_THRU_4,
        register_map_tag::DISTANCE_3_THRU_0,
    };

    if (!burst_read_from_register(registers_to_read, register_entries, sizeof(registers_to_read)))
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

[[nodiscard]] uint32_t gp2y0e02b::distance_sensor::get_read_delay(const register_map_tag reg) const
{
    //TODO Delay may be longer or shorter depending on tag and state
    return READ_DELAY_DFT_MS;
}

std::optional<gp2y0e02b::register_map_entry> gp2y0e02b::distance_sensor::read_from_register(register_map_tag reg) const
{
    uint8_t target_register = static_cast<uint8_t>(reg);
    uint8_t addr_write = get_write_addr();

    i2c_operation_job_t ops_0[] = {
        { .command = I2C_MASTER_CMD_START },    //I2C Start Cycle 1
        {                                       //Select address for write
            .command = I2C_MASTER_CMD_WRITE,
            .write = {
                .ack_check = true,
                .data = &addr_write,
                .total_bytes = 1
            }
        },
        {                                       //Select register
            .command = I2C_MASTER_CMD_WRITE,
                .write = {
                .ack_check = true,
                .data = &target_register,
                .total_bytes = 1
            }
        },
        { .command = I2C_MASTER_CMD_STOP },    //I2C Stop Cycle 1
    };

    uint8_t addr_read = get_read_addr();
    uint8_t buffer_read = 0;

    i2c_operation_job_t ops_1[] = {
        { .command = I2C_MASTER_CMD_START },    //I2C Start Cycle 2
        {                                       //Select address for read
            .command = I2C_MASTER_CMD_WRITE,
            .write = {
                .ack_check = true,
                .data = &addr_read,
                .total_bytes = 1
            }
        },
        {                                       //Read data
            .command = I2C_MASTER_CMD_READ,
            .read = {
                .ack_value = I2C_NACK_VAL,
                .data = &buffer_read,
                .total_bytes = 1
            }
        },
        { .command = I2C_MASTER_CMD_STOP }      //I2C Stop Cycle 2
    };

    const auto response_select_register = i2c_master_execute_defined_operations(
        handle->handle,
        ops_0, sizeof(ops_0) / sizeof(i2c_operation_job_t),
        timeout_ms
    );

    if (response_select_register != ESP_OK)
        return std::nullopt;

    const auto response_read_register = i2c_master_execute_defined_operations(
        handle->handle,
        ops_1, sizeof(ops_1) / sizeof(i2c_operation_job_t),
        timeout_ms
    );

    if (response_read_register != ESP_OK)
        return std::nullopt;

    return register_map_entry
    {
        .tag = reg,
        .data = {
            .raw_value = buffer_read
        }
    };
}

bool gp2y0e02b::distance_sensor::burst_read_from_register(const register_map_tag registers[], register_map_entry results[], const size_t read_len) const
{
    assert(read_len > 0);

    //If read length is 1, call standard read from register
    if (read_len == 1)
    {
        if (const auto maybe_result = read_from_register(registers[0]); maybe_result.has_value())
        {
            results[0] = maybe_result.value();
            return true;
        }

        return false;
    }

    //Registers MUST be adjacent for burst read.
    auto prev = static_cast<uint8_t>(register_map_tag::UNKNOWN);
    for (size_t i = 0; i < read_len; i++)
    {
        const auto current_tag = registers[i];
        const auto current_tag_addr = static_cast<uint8_t>(current_tag);

        if (i > 0)
            assert(current_tag_addr == prev + 1);

        prev = current_tag_addr;
    }

    uint8_t target_register = static_cast<uint8_t>(registers[0]);
    uint8_t addr_write = get_write_addr();

    i2c_operation_job_t ops_0[] = {
        { .command = I2C_MASTER_CMD_START },    //I2C Start Cycle 1
        {                                       //Select address for write
            .command = I2C_MASTER_CMD_WRITE,
            .write = {
                .ack_check = true,
                .data = &addr_write,
                .total_bytes = 1
            }
        },
        {                                       //Select start register
            .command = I2C_MASTER_CMD_WRITE,
                .write = {
                    .ack_check = true,
                    .data = &target_register,
                    .total_bytes = 1
                }
        },
        { .command = I2C_MASTER_CMD_STOP },    //I2C Stop Cycle 1
    };

    uint8_t addr_read = get_read_addr();
    const auto buffer_read = std::make_unique<uint8_t[]>(read_len); //alloc read buffer that will be freed at end of scope

    i2c_operation_job_t ops_1[] = {
        { .command = I2C_MASTER_CMD_START },    //I2C Start Cycle 2
        {                                       //Select address for read
            .command = I2C_MASTER_CMD_WRITE,
            .write = {
                .ack_check = true,
                .data = &addr_read,
                .total_bytes = 1
            }
        },
        {                                       //Read data
            .command = I2C_MASTER_CMD_READ,
            .read = {
                .ack_value = I2C_NACK_VAL,
                .data = buffer_read.get(),
                .total_bytes = read_len
            }
        },
        { .command = I2C_MASTER_CMD_STOP }      //I2C Stop Cycle 2
    };

    const auto response_select_register = i2c_master_execute_defined_operations(
        handle->handle,
        ops_0, sizeof(ops_0) / sizeof(i2c_operation_job_t),
        timeout_ms
    );

    if (response_select_register != ESP_OK)
        return false;

    //Delay between selecting register and reading
    vTaskDelay(get_read_delay(registers[0]) / portTICK_PERIOD_MS);

    const auto response_read_register = i2c_master_execute_defined_operations(
        handle->handle,
        ops_1, sizeof(ops_1) / sizeof(i2c_operation_job_t),
        timeout_ms
    );

    if (response_read_register != ESP_OK)
        return false;

    //Populate results
    for (size_t i = 0; i < read_len; i++)
    {
        results[i] = register_map_entry
        {
            .tag = registers[i],
            .data = {
                .raw_value = buffer_read[i]
            }
        };
    }

    return true;
}
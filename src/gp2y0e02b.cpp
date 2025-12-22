//
// Created by bendstein on 12/20/2025.
//
#include "app_common.h"
#include "gp2y0e02b.h"

#include <driver/i2c.h>
#include <driver/i2c_master.h>

void gp2y0e02b::set_i2c_addr(i2c_master_bus_handle_t bus, uint8_t addr_new)
{
    //Per docs, only top 4 bits are available for addressing.
    //Bottom four bits must be 0 for this purpose.
    assert((addr_new & 0x0F) == 0);
    assert(bus != nullptr);

    LOGI("gp2y0e02b", std::format("Programming sensor efuses to use address 0x{:02X}.", addr_new));

    //Connect to device
    const auto device = i2c_init_device(bus, GP2Y0E02B_I2C_ADDR_DFT, GP2Y0E02B);

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

[[nodiscard]]
bool gp2y0e02b::ping(const i2c_device* device, const int timeout_ms)
{
    constexpr uint8_t register_hold_bit = 0x03;
    const auto hold_bit_data = read_from_register(
        device,
        register_hold_bit,
        timeout_ms);

    return hold_bit_data.has_value();

    // constexpr uint8_t write_buffer[] = { 0x80 };
    // uint8_t read_buffer[1] = {};
    //
    // const auto response_status = i2c_master_transmit_receive(
    //     device->handle,
    //     write_buffer, sizeof(write_buffer),
    //     read_buffer, sizeof(read_buffer),
    //     timeout_ms
    // );

    // constexpr uint8_t reg_addr_hold_bit = 0x03;
    // uint8_t resp_hold_bit = 0x00;
    //
    // const auto response_status = i2c_master_transmit_receive(
    //     device->handle,
    //     &reg_addr_hold_bit, sizeof(reg_addr_hold_bit),
    //     &resp_hold_bit, sizeof(resp_hold_bit),
    //     timeout_ms
    // );

    // return response_status == ESP_OK;

    // uint8_t ping_addr = GP2Y0E02B_ADDR_AS_WRITE(device->address);
    // uint8_t reg_addr_hold_bit = 0x03;
    //
    // i2c_master_transmit_multi_buffer_info_t message[] = {
    //     {
    //         .write_buffer = &ping_addr,
    //         .buffer_size = sizeof(ping_addr)
    //     },
    //     {
    //         .write_buffer = &reg_addr_hold_bit,
    //         .buffer_size = sizeof(reg_addr_hold_bit)
    //     }
    // };
    //
    // const auto write__select_reg_hold_bit = i2c_master_multi_buffer_transmit(
    //     device->handle,
    //     message,
    //     sizeof(message) / sizeof(i2c_master_transmit_multi_buffer_info_t),
    //     timeout_ms
    // );
    //
    // if (write__select_reg_hold_bit != ESP_OK)
    //     return false;
    //
    // const auto status = i2c_master_transmit(
    //     device->handle,
    //     &ping_addr,
    //     1,
    //     timeout_ms);
    //
    // return status == ESP_OK;
}

[[nodiscard]]
std::optional<uint8_t> gp2y0e02b::read_from_register(const i2c_device* device, uint8_t reg, const int timeout_ms)
{
    if (device->handle == nullptr) return std::nullopt;
    if (device->type != GP2Y0E02B) return std::nullopt;

    uint8_t addr_write = GP2Y0E02B_ADDR_AS_WRITE(device->address);

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
                .data = &reg,
                .total_bytes = 1
            }
        },
        { .command = I2C_MASTER_CMD_STOP },    //I2C Stop Cycle 1
    };

    uint8_t addr_read = GP2Y0E02B_ADDR_AS_READ(device->address);
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
        device->handle,
        ops_0, sizeof(ops_0) / sizeof(i2c_operation_job_t),
        timeout_ms
    );

    if (response_select_register != ESP_OK)
        return std::nullopt;

    const auto response_read_register = i2c_master_execute_defined_operations(
        device->handle,
        ops_1, sizeof(ops_1) / sizeof(i2c_operation_job_t),
        timeout_ms
    );

    if (response_read_register != ESP_OK)
        return std::nullopt;

    return buffer_read;
}
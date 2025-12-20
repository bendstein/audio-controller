//
// Created by bendstein on 12/4/2025.
//
#include "sensor_cfg.h"
#include "app_common.h"

#include <cassert>
#include <driver/i2c.h>
#include <driver/i2c_master.h>

void set_gp2y0e02b_i2c_addr(const i2c_master_bus_handle_t bus, const uint8_t addr_new)
{
    //Per docs, only top 4 bits are available for addressing.
    //Bottom four bits must be 0 for this purpose.
    assert((addr_new & 0x0F) == 0);
    assert(bus != nullptr);

    LOGI("i2c", std::format("Programming sensor efuses to use address {:04X}.", addr_new));

    //Connect to device
    const auto handle = i2c_init_device(bus, GP2Y0E02B_I2C_ADDR_DFT);

    return;

    vTaskDelay(1 / portTICK_PERIOD_MS); //Wait 1ms for Vcc

    //Stage 1

    //Clock Select (0xEC) := Manual (0xFF)
    constexpr uint8_t buffer_clock_sel[] = { 0xEC, 0xFF };
    ESP_ERROR_CHECK(i2c_master_transmit(handle, buffer_clock_sel, sizeof(buffer_clock_sel), -1));

    //Enable Vpp
    gpio_set_level(PIN_ENABLE_VPP, HIGH);
    vTaskDelay(1 / portTICK_PERIOD_MS); //Wait 1ms for Vpp

    //Stage 2

    //Target addr (0xC8) := 0x00 (lsb of peripheral id in bank e)
    constexpr uint8_t buffer_target_lsb_sel[] = { 0xC8, 0x00 };
    ESP_ERROR_CHECK(i2c_master_transmit(handle, buffer_target_lsb_sel, sizeof(buffer_target_lsb_sel), -1));

    //Stage 3

    //Target bit/bank (0xC9) := 0x45 (Specify the bits/bank to set values for)
    constexpr uint8_t buffer_target_bits_sel[] = { 0xC9, 0x45 };
    ESP_ERROR_CHECK(i2c_master_transmit(handle, buffer_target_bits_sel, sizeof(buffer_target_bits_sel), -1));

    //Stage 4

    //Data (0xCD) := addr_new (Set data)
    //>> 4 bc cannot program bottom 4 bits (handled by assert at top)
    const uint8_t buffer_set_data[] = { 0xCA, static_cast<uint8_t>(addr_new >> 4) };
    ESP_ERROR_CHECK(i2c_master_transmit(handle, buffer_set_data, sizeof(buffer_set_data), -1));

    //Stage 5

    //Program Enable (0xCA) := 0x01 (Start commit)
    constexpr uint8_t buffer_write_program_start[] = { 0xCA, 0x01 };
    ESP_ERROR_CHECK(i2c_master_transmit(handle, buffer_write_program_start, sizeof(buffer_write_program_start), -1));

    vTaskDelay(500 / portTICK_PERIOD_US); //Delay 500 microseconds while writing program

    //Stage 6

    //Program Enable (0xCA) := 0x00 (End commit)
    constexpr uint8_t buffer_write_program_end[] = { 0xCA, 0x00 };
    ESP_ERROR_CHECK(i2c_master_transmit(handle, buffer_write_program_end, sizeof(buffer_write_program_end), -1));

    //Disable Vpp
    gpio_set_level(PIN_ENABLE_VPP, LOW);
    vTaskDelay(1 / portTICK_PERIOD_MS); //Wait 1ms for Vpp

    //Stage 7

    constexpr uint8_t buffer_stage_7_0[] = { 0xEF, 0x00 }; //Bank Select (0xEF) := 0x00 (Digital control register)
    constexpr uint8_t buffer_stage_7_1[] = { 0xC8, 0x40 }; //E-Fuse read out (0xC8:6) := 0x40 (Load E-Fuse data to register)
    constexpr uint8_t buffer_stage_7_2[] = { 0xC8, 0x00 }; //E-Fuse read out (0xC8:6) := 0x00 (Unset prev)

    ESP_ERROR_CHECK(i2c_master_transmit(handle, buffer_stage_7_0, sizeof(buffer_stage_7_0), -1));
    ESP_ERROR_CHECK(i2c_master_transmit(handle, buffer_stage_7_1, sizeof(buffer_stage_7_1), -1));
    ESP_ERROR_CHECK(i2c_master_transmit(handle, buffer_stage_7_2, sizeof(buffer_stage_7_2), -1));

    //Stage 8

    //Software reset (0xEE) := 0x06 (Reset)
    constexpr uint8_t buffer_stage_8[] = { 0xEE, 0x06 };
    ESP_ERROR_CHECK(i2c_master_transmit(handle, buffer_stage_8, sizeof(buffer_stage_8), -1));

    // //Stage 9
    //
    // constexpr uint8_t buffer_stage_9_0[] = { 0xEF, 0x00 }; //Bank Select (0xEF) := 0x00 (Digital control register)
    // constexpr uint8_t buffer_stage_9_1[] = { 0xEC, 0xFF }; //Clock Select (0xEC) := Manual (0xFF)
    // constexpr uint8_t buffer_stage_9_2[] = { 0xEF, 0x03 }; //Bank select (0xEF) := 0x03 (E-fuse register)
    //
    // ESP_ERROR_CHECK(i2c_master_transmit(handle, buffer_stage_9_0, sizeof(buffer_stage_9_0), -1));
    // ESP_ERROR_CHECK(i2c_master_transmit(handle, buffer_stage_9_1, sizeof(buffer_stage_9_1), -1));
    // ESP_ERROR_CHECK(i2c_master_transmit(handle, buffer_stage_9_2, sizeof(buffer_stage_9_2), -1));
    //
    //
}

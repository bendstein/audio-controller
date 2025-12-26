//
// Created by bendstein on 12/20/2025.
//

#ifndef AUDIO_CONTROLLER_GP2Y0E02B_H
#define AUDIO_CONTROLLER_GP2Y0E02B_H
#include <driver/i2c_master.h>

#include "gp2y0e02b_register_map.h"
#include "i2c.h"

namespace gp2y0e02b
{
    struct distance_sensor_state
    {
        uint8_t distance;
        shift_bit distance_shift;

        /**
         * Set values back to default
         */
        void reset()
        {
            distance = 0;
            distance_shift = shift_bit::cm_64;
        }
    };

    class distance_sensor
    {
    public:
        static constexpr uint8_t I2C_ADDR_DFT = 0x80;
        static constexpr gpio_num_t PIN_VPP_ENABLE = GPIO_NUM_14;
    private:
        i2c_master_dev_handle_t handle;
        int32_t timeout_ms = -1;
        distance_sensor_state state {};
        uint8_t address;
    public:
        explicit distance_sensor(i2c_master_dev_handle_t device_handle, const uint8_t address)
            : handle(device_handle), address(address)
        {
            assert(device_handle != nullptr);
            state.reset();
        }

        distance_sensor(i2c_master_dev_handle_t device_handle, const uint8_t address, const int32_t timeout)
            : distance_sensor(device_handle, address)
        {
            timeout_ms = timeout;
        }

        [[nodiscard]] int32_t get_timeout_ms() const { return timeout_ms; }
        void set_timeout(const int32_t timeout) { timeout_ms = timeout; }

        /**
         * @return Current state of this value
         * @remark Doesn't query sensor register, uses cached value. Use corresponding update_*
         *         method to update cached value.
         */
        [[nodiscard]] uint8_t get_distance() const { return state.distance; }

        /**
         * @return Current state of this value
         * @remark Doesn't query sensor register, uses cached value. Use corresponding update_*
         *         method to update cached value.
         */
        [[nodiscard]] shift_bit get_distance_shift() const { return state.distance_shift; }

        /**
         * @return Whether the sensor is alive
         */
        [[nodiscard]] bool ping() const;

        /**
         * @param distance_out New state of this value. Ignored if nullptr.
         * @param prev_distance_out Previous state of this value. Ignored if nullptr.
         * @return Whether the action was successful.
         * @remark Side effect: Update cached state of this value
         */
        [[nodiscard]] bool try_update_distance(uint8_t* distance_out = nullptr, uint8_t* prev_distance_out = nullptr);

        /**
         * @param distance_shift_out New state of this value. Ignored if nullptr.
         * @param prev_distance_shift_out Previous state of this value. Ignored if nullptr.
         * @return Whether the action was successful.
         * @remark Side effect: Update cached state of this value
         */
        [[nodiscard]] bool try_update_distance_shift(shift_bit* distance_shift_out = nullptr, shift_bit* prev_distance_shift_out = nullptr);

        /**
         * Write the new shift bit to the sensor register, and update cached value.
         * @param new_shift_bit The value to write to the register
         * @param prev_distance_shift_out Previous state of this value. Ignored if nullptr.
         * @return Whether the action was successful.
         * @remark Side effect: Update cached state of this value
         */
        [[nodiscard]] bool try_apply_distance_shift(shift_bit new_shift_bit, shift_bit* prev_distance_shift_out = nullptr);

        /**
         * Try performing a software reset
         * @return Whether the operation was successful
         * @remark Side effect: restarts device, resets cached state
         */
        [[nodiscard]] bool try_soft_reset();

        /**
         * @param entry The register entry to read
         * @return Whether the read was successful
         * @remark Side effect: populates entry->data
         */
        [[nodiscard]] bool try_read_from_register(register_map_entry* entry) const;

        /**
         * @param entries The register entries to read
         * @param read_len The length of the entries array
         * @return Whether the read was successful
         * @remark Side effect: populates each entries[i]->data
         */
        [[nodiscard]] bool try_burst_read_from_register (register_map_entry entries[], size_t read_len) const;

        /**
         * @param entry The register entry to write
         * @return Whether write was successful
         * @remark Side effect: updates the value in the sensor's register
         * @warning This action is destructive if writing to one of the
         *          e-fuse registers, as once an e-fuse is burnt, it
         *          can no longer be changed
         */
        [[nodiscard]] bool try_write_to_register(const register_map_entry* entry) const;

        /**
         * Initialize distance sensor, creating a device handle and adding it to the bus
         * @param bus Handle for the i2c bus
         * @param addr i2c address for this sensor
         * @param timeout_ms standard timeout in milliseconds for i2c operations
         * @return The created distance sensor, or none if failed
         * @remark Adds a new device to the master bus
         */
        [[nodiscard]] static std::optional<distance_sensor*> try_create_on_bus(i2c_master_bus_handle_t bus, const uint8_t addr, const int32_t timeout_ms)
        {
            const i2c_device_config_t device_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = static_cast<uint8_t>(addr >> 1),
                .scl_speed_hz = I2C_DEVICE_SCL_SPEED_HZ,
                .scl_wait_us = I2C_DEVICE_SCL_WAIT_US,
                .flags = {
                    .disable_ack_check = false
                }
            };

            i2c_master_dev_handle_t handle;

            if (const auto add_to_bus_result = i2c_master_bus_add_device(bus, &device_cfg, &handle); add_to_bus_result != ESP_OK)
                return std::nullopt;

            return new distance_sensor(handle, addr, timeout_ms);
        }

        ~distance_sensor()
        {
            //Make sure to remove the sensor from
            //the i2c bus when it is destroyed.
            //Technically this can fail, but not
            //sure what to do in that case
            i2c_master_bus_rm_device(handle);
        }

        /**
         * Permanently programs the e-fuses on the sensor to change its
         * i2c address.
         * @param bus The master bus that the sensor is on
         * @param addr_new The new address to write to the device
         * @warning This action is destructive, as once an e-fuse
         *          is burnt, it can no longer be changed.
         */
        static void permanently_apply_new_i2c_address(i2c_master_bus_handle_t bus, uint8_t addr_new)
        {
            //Per docs, only top 4 bits are available for addressing.
            //Bottom four bits must be 0 for this purpose.
            assert((addr_new & 0x0F) == 0);
            assert(bus != nullptr);

            const auto maybe_sensor = try_create_on_bus(bus, I2C_ADDR_DFT, -1);

            if (!maybe_sensor.has_value())
                throw std::runtime_error("Failed to create sensor");

            const auto sensor = *maybe_sensor;

            logi(NAMEOF(gp2y0e02b), std::format("Programming sensor e-fuses to use address 0x{:02X}.", addr_new));

            //The steps below are taken from the datasheet, table 20.

            vTaskDelay(1 / portTICK_PERIOD_MS); //Wait 1ms for Vcc

            //Stage 1

            //Set clock to manual
            logi(NAMEOF(gp2y0e02b), "Step 1: set clock to manual");
            constexpr register_map_entry clock_select_entry = {
                .tag = register_map_tag::CLOCK_SELECT,
                .data = {
                    .clock_select = {
                        .clock = clock_select::clk_manual
                    }
                }
            };

            if (!sensor->try_write_to_register(&clock_select_entry))
                throw std::runtime_error("Failed to write clock select");

            //Enable Vpp
            logi(NAMEOF(gp2y0e02b), "Step 1.1: Set Vpp to high");
            gpio_set_level(PIN_VPP_ENABLE, HIGH);
            vTaskDelay(1 / portTICK_PERIOD_MS); //Wait 1ms for Vpp

            //Stage 2
            //Select e-fuse target (haven't selected bank yet, but
            //this is to set target to the very start of bank e,
            //which is where the i2c address is set)
            logi(NAMEOF(gp2y0e02b), "Step 2: Select target address for update in e-fuse bank.");
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

            if (!sensor->try_write_to_register(&efuse_target_entry))
                throw std::runtime_error("Failed to select target e-fuse");

            //Stage 3
            //Specify the bank to write in, and the bit number.
            //i2c address is in bank e.
            //As far as I can tell, we use 0x04 because that's the
            //length of the i2c address in memory, with lsb at 0x00
            //set in the previous write.
            //The process described in the datasheet is unclear, so
            //this is just my best guess.
            logi(NAMEOF(gp2y0e02b), "Step 3: Select e-fuse bank and bit number for update.");
            constexpr register_map_entry bank_target_entry = {
                .tag = register_map_tag::EFUSE_BIT_NUMBER_AND_BANK_ASSIGN,
                .data = {
                    .efuse_bit_number_and_bank_assign = {
                        .bank_select = register_bank::bank_e,
                        .bit_number = 0x04,
                    }
                }
            };

            if (!sensor->try_write_to_register(&bank_target_entry))
                throw std::runtime_error("Failed to select target e-fuse bank");

            //Stage 4

            //Set new address in efuse program register
            logi(NAMEOF(gp2y0e02b), "Step 4: Set new i2c address in efuse program register.");
            const register_map_entry set_new_address_entry = {
                .tag = register_map_tag::EFUSE_PROGRAM_DATA,
                .data = {
                    .efuse_program_data = {
                        .data = static_cast<uint8_t>(addr_new >> 4) //>> 4 bc cannot program bottom 4 bits
                    }
                }
            };

            if (!sensor->try_write_to_register(&set_new_address_entry))
                throw std::runtime_error("Failed to set new address in e-fuse program register");

            //Stage 5

            //Enable programming (start commit)
            logi(NAMEOF(gp2y0e02b), "Step 5, 6: Enable e-fuse program, wait 500us, and then disable e-fuse program, to apply changes.");
            constexpr register_map_entry start_program_entry = {
                .tag = register_map_tag::EFUSE_PROGRAM_ENABLE_BIT,
                .data = {
                    .efuse_program_enable_bit = {
                        .enable = efuse_program_enable_bit::enable
                    }
                }
            };

            if (!sensor->try_write_to_register(&start_program_entry))
                throw std::runtime_error("Failed to enable/start e-fuse program");

            vTaskDelay(500 / portTICK_PERIOD_US); //Delay 500 microseconds while writing program

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

            if (!sensor->try_write_to_register(&end_program_entry))
                throw std::runtime_error("Failed to disable/end e-fuse program");

            logi(NAMEOF(gp2y0e02b), "Step 6.1: Finished applying e-fuse program. Bringing Vpp back to low.");

            //Disable Vpp
            gpio_set_level(PIN_VPP_ENABLE, LOW);
            vTaskDelay(1 / portTICK_PERIOD_MS); //Wait 1ms for Vpp

            //Stage 7

            //Change bank to control register
            logi(NAMEOF(gp2y0e02b), "Step 7: Selecting control register/bank a.");
            constexpr register_map_entry select_control_register_entry = {
                .tag = register_map_tag::BANK_SELECT,
                .data = {
                    .bank_select = {
                        .bank = bank_select::select_bank_a
                    }
                }
            };

            if (!sensor->try_write_to_register(&select_control_register_entry))
                throw std::runtime_error("Failed to select control register");

            //Update register values from e-fuses
            logi(NAMEOF(gp2y0e02b), "Step 7.1: enable loading e-fuse data into register.");
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

            if (!sensor->try_write_to_register(&enable_update_register_entry))
                throw std::runtime_error("Failed to update register data from e-fuses");

            //Stop updating register values from e-fuses
            logi(NAMEOF(gp2y0e02b), "Step 7.2: disable loading e-fuse data into register.");
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

            if (!sensor->try_write_to_register(&disable_update_register_entry))
                throw std::runtime_error("Failed to stop updating register data from e-fuses");

            //Stage 8

            //Software reset
            logi(NAMEOF(gp2y0e02b), "Step 8: Restarting sensor.");

            constexpr register_map_entry reset_software_entry = {
                .tag = register_map_tag::SOFTWARE_RESET,
                .data = {
                    .software_reset = {
                        ._dummy = 0,
                        .reset = software_reset::reset
                    }
                }
            };

            if (!sensor->try_write_to_register(&reset_software_entry))
                throw std::runtime_error("Failed to restart sensor");

            //Stages 9, 10 are validation and error-handling.
        }
    };
}

#endif //AUDIO_CONTROLLER_GP2Y0E02B_H
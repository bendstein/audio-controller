//
// Created by bendstein on 12/20/2025.
//

#ifndef AUDIO_CONTROLLER_GP2Y0E02B_H
#define AUDIO_CONTROLLER_GP2Y0E02B_H
#include <memory>

#include <driver/i2c_master.h>

#include "register_map.h"

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
        static constexpr uint32_t I2C_SCL_WAIT_US_DFT = 0;
        static constexpr gpio_num_t PIN_VPP_ENABLE = GPIO_NUM_14;
        static constexpr int32_t TIMEOUT_MS_DFT = 5000;
    private:
        i2c_master_dev_handle_t handle;
        int32_t timeout_ms = -1;
        distance_sensor_state state {};
        uint8_t address;
        size_t index;
        std::string log_key;

        [[nodiscard]] bool try_select_register(register_map_tag tag) const;
        [[nodiscard]] std::string make_log_key() const;
    public:
        distance_sensor(i2c_master_dev_handle_t device_handle, uint8_t address, size_t index);
        distance_sensor(i2c_master_dev_handle_t device_handle, uint8_t address, size_t index, int32_t timeout);

        distance_sensor() = delete;
        distance_sensor(const distance_sensor& other) = delete;
        distance_sensor(distance_sensor&& other) = delete;
        distance_sensor& operator=(const distance_sensor& other) = delete;
        distance_sensor& operator=(distance_sensor&& other) = delete;

        [[nodiscard]] int32_t get_timeout_ms() const { return timeout_ms; }
        void set_timeout(const int32_t timeout) { timeout_ms = timeout; }

        [[nodiscard]] uint8_t get_addr() const { return address; }
        [[nodiscard]] size_t get_index() const { return index; }

        [[nodiscard]] const std::string& get_log_key() const { return log_key; }

        /**
         * @return Current state of this value
         * @remark Doesn't query sensor register, uses cached value. Use corresponding update_*
         *         method to update cached value from device.
         */
        [[nodiscard]] uint8_t get_distance() const { return state.distance; }

        /**
         * @param new_distance The new distance to set
         * @remark Doesn't touch sensor registers, only updates cached value.
         */
        void set_distance(const uint8_t new_distance) { state.distance = new_distance; }

        /**
         * @return Current state of this value
         * @remark Doesn't query sensor register, uses cached value. Use corresponding update_*
         *         method to update cached value.
         */
        [[nodiscard]] shift_bit get_distance_shift() const { return state.distance_shift; }

        [[nodiscard]] uint8_t get_distance_shift_value() const
        {
            switch (get_distance_shift())
            {
                case shift_bit::cm_128:
                    return SHIFT_BIT_DISTANCE_128;
                case shift_bit::cm_64:
                default:
                    return SHIFT_BIT_DISTANCE_64;
            }
        }

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
        [[nodiscard]] bool try_burst_read_from_register(register_map_entry entries[], size_t read_len) const;

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
         * @param index incrementing id for sensor
         * @param timeout_ms standard timeout in milliseconds for i2c operations
         * @return The created distance sensor, or none if failed
         * @remark Adds a new device to the master bus
         */
        [[nodiscard]] static std::unique_ptr<distance_sensor> try_create_on_bus(i2c_master_bus_handle_t bus, uint8_t addr, size_t index, int32_t timeout_ms);

        /**
         * Permanently programs the e-fuses on the sensor to change its
         * i2c address.
         * @param bus The master bus that the sensor is on
         * @param addr_new The new address to write to the device
         * @warning This action is destructive, as once an e-fuse
         *          is burnt, it can no longer be changed.
         */
        static void permanently_apply_new_i2c_address(i2c_master_bus_handle_t bus, uint8_t addr_new);

        ~distance_sensor();
    };
}

#endif //AUDIO_CONTROLLER_GP2Y0E02B_H
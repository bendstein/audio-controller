//
// Created by bendstein on 12/20/2025.
//

#ifndef AUDIO_CONTROLLER_GP2Y0E02B_H
#define AUDIO_CONTROLLER_GP2Y0E02B_H
#include "gp2y0e02b_register_map.h"
#include "i2c.h"

#define GP2Y0E02B_I2C_ADDR_DFT 0x80
#define GP2Y0E02B_PIN_ENABLE_VPP gpio_num_t::GPIO_NUM_14

//Addr for read is top 7 bits, with LSB 1
#define GP2Y0E02B_ADDR_AS_READ(addr) (((addr) & 0xFE) | 0x01)

//Addr for write is top 7 bits, with LSB 0
#define GP2Y0E02B_ADDR_AS_WRITE(addr) ((addr) & 0xFE)

namespace gp2y0e02b
{
    void set_i2c_addr(i2c_master_bus_handle_t bus, uint8_t addr_new);

    struct distance_sensor_state
    {
        uint8_t distance;
        shift_bit distance_shift;
    };

    class distance_sensor
    {
    public:
        static constexpr uint8_t I2C_ADDR_DFT = 0x80;
        static constexpr gpio_num_t PIN_VPP_ENABLE = GPIO_NUM_14;
        static constexpr uint32_t READ_DELAY_DFT_MS = 100;

    private:
        const i2c_device* handle;
        uint32_t timeout_ms = 0;
        distance_sensor_state state {};
    public:
        explicit distance_sensor(const i2c_device* device_handle)
        {
            assert(device_handle != nullptr);
            assert(device_handle->type == i2c_device_type::GP2Y0E02B);
            handle = device_handle;
        }

        distance_sensor(const i2c_device* device_handle, const uint32_t timeout) : distance_sensor(device_handle)
        {
            timeout_ms = timeout;
        }

        [[nodiscard]] uint32_t get_timeout_ms() const { return timeout_ms; }
        void set_timeout(const uint32_t timeout) { timeout_ms = timeout; }

        [[nodiscard]] uint8_t get_write_addr() const { return handle->address & 0xFE; }
        [[nodiscard]] uint8_t get_read_addr() const { return (handle->address & 0xFE) | 1; }

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
         * @param reg
         * @return The time that must be waited between selecting the register and reading it
         */
        [[nodiscard]] uint32_t get_read_delay(register_map_tag reg) const;

        [[nodiscard]] bool ping() const;

        /**
         * @param distance_out New state of this value. Ignored if nullptr.
         * @param prev_distance_out Previous state of this value. Ignored if nullptr.
         * @return Whether the action was successful.
         * @remark Side effect: Update cached state of this value
         */
        bool update_distance(uint8_t* distance_out = nullptr, uint8_t* prev_distance_out = nullptr);

        /**
         * @param distance_shift_out New state of this value. Ignored if nullptr.
         * @param prev_distance_shift_out Previous state of this value. Ignored if nullptr.
         * @return Whether the action was successful.
         * @remark Side effect: Update cached state of this value
         */
        bool update_distance_shift(shift_bit* distance_shift_out = nullptr, shift_bit* prev_distance_shift_out = nullptr);

        [[nodiscard]] std::optional<register_map_entry> read_from_register(register_map_tag reg) const;

        [[nodiscard]] bool burst_read_from_register (const register_map_tag registers[], register_map_entry results[], size_t read_len) const;
    };
}

#endif //AUDIO_CONTROLLER_GP2Y0E02B_H
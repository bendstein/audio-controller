//
// Created by bendstein on 12/3/2025.
//

#ifndef AUDIO_CONTROLLER_I2C_H
#define AUDIO_CONTROLLER_I2C_H
#include <memory>

#include <soc/gpio_num.h>
#include <driver/i2c_master.h>

constexpr auto I2C_STANDARD_HZ = 100000;
constexpr auto I2C_FAST_HZ = 400000;
constexpr auto I2C_VERY_FAST_HZ = 1000000; //Non-standard
constexpr auto I2C_HIGH_SPEED_HZ = 34000000;

struct i2c_master_create_cfg
{
    uint8_t port;
    uint8_t scl;
    uint8_t sda;
};

class i2c_master
{
public:
    static constexpr auto GLITCH_CT_DFT = 7;
    static constexpr auto INTERRUPT_PRIORITY_DFT = 0;
    static constexpr auto TRANS_QUEUE_DEPTH_DFT = 0;
    static constexpr auto INTERNAL_PULLUP_DFT = 1;
    static constexpr auto ALLOW_SLEEP_DFT = 0;
private:
    const i2c_master_create_cfg options;
    i2c_master_bus_handle_t handle;
    std::string log_key;

    [[nodiscard]] std::string make_log_key() const;
public:
    i2c_master(i2c_master_create_cfg options, i2c_master_bus_handle_t handle);

    i2c_master() = delete;
    i2c_master(const i2c_master& other) = delete;
    i2c_master(i2c_master&& other) = delete;
    i2c_master& operator=(const i2c_master& other) = delete;
    i2c_master& operator=(i2c_master&& other) = delete;

    ~i2c_master();

    [[nodiscard]] i2c_master_bus_handle_t get_handle() const { return handle; }
    [[nodiscard]] i2c_port_t get_port() const { return static_cast<i2c_port_t>(options.port); }
    [[nodiscard]] gpio_num_t get_sda() const { return static_cast<gpio_num_t>(options.sda); }
    [[nodiscard]] gpio_num_t get_scl() const { return static_cast<gpio_num_t>(options.scl); }
    [[nodiscard]] const std::string& get_log_key() const { return log_key; }

    [[nodiscard]] static std::unique_ptr<i2c_master> try_create(i2c_master_create_cfg create_options);
};

#endif //AUDIO_CONTROLLER_I2C_H
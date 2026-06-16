//
// Created by bendstein on 6/14/2026.
//

#ifndef AUDIO_CONTROLLER_I2C_BUS_H
#define AUDIO_CONTROLLER_I2C_BUS_H
#include <map>
#include <soc/gpio_num.h>

namespace i2c
{
    static constexpr uint8_t ADDR_DEV_NONE = 0xFF;

    struct bus_cfg
    {
        gpio_num_t sda = GPIO_NUM_NC;
        gpio_num_t scl = GPIO_NUM_NC;
    };

    struct dev_cfg
    {
        uint8_t address = ADDR_DEV_NONE;
    };

    struct i_message
    {
        virtual const uint8_t* to_message(size_t& size) const;

        virtual ~i_message() = default;
    };

    enum struct result_type
    {
        OK = 0,
        ACK = OK,
        NACK = 1,
        GENERAL_ERROR = 2,
        TIMEOUT = 3
    };

    struct result
    {
        static constexpr auto DEFAULT_ERROR_MESSAGE = "An unknown error has occurred.";

        result_type type = result_type::OK;
        std::optional<const std::string> error_message = std::nullopt;
        std::optional<uint8_t> data = std::nullopt;

        result() = default;
        explicit result(const result_type& type) : type(type) {}
        result(const result_type& type, const std::string& message) : type(type), error_message(message) {}
        result(const result_type& type, const uint8_t data) : type(type), data(data) {}

        [[nodiscard]] uint8_t data_or_default() const
        {
            if (is_ok())
            {
                return data.has_value()
                    ? data.value()
                    : 0;
            }

            return 0;
        }

        [[nodiscard]] std::string error_message_or_default() const
        {
            if (is_err())
            {
                return error_message.has_value()
                    ? error_message.value()
                    : DEFAULT_ERROR_MESSAGE;
            }

            return "";
        }

        [[nodiscard]] bool is_ok() const noexcept { return type == result_type::OK; }
        [[nodiscard]] bool is_ok(uint8_t& value) const noexcept
        {
            if (is_ok())
            {
                value = data.has_value()
                    ? data.value()
                    : 0;
                return true;
            }

            value = 0;
            return false;
        }
        [[nodiscard]] bool is_err() const noexcept { return type != result_type::OK; }
        [[nodiscard]] bool is_err(std::string& message) const noexcept
        {
            if (is_err())
            {
                message = error_message.has_value()
                    ? error_message.value()
                    : DEFAULT_ERROR_MESSAGE;
                return true;
            }

            message = "";
            return false;
        }

        static result ok() noexcept { return result {}; }
        static result ok(const uint8_t data) noexcept { return result { result_type::OK, data }; }
        static result ack() noexcept { return result { result_type::ACK }; }
        static result nack() noexcept { return result { result_type::NACK }; }
        static result general_error() noexcept { return result { result_type::GENERAL_ERROR }; }
        static result general_error(const std::string& message) noexcept { return result { result_type::GENERAL_ERROR, message }; }
        static result timeout() noexcept { return result { result_type::TIMEOUT }; }
        static result timeout(const std::string& message) noexcept { return result { result_type::TIMEOUT, message }; }
    };
    
    class device
    {
        dev_cfg cfg {};
    public:
        device() = default;
        explicit device(const dev_cfg& cfg) : cfg(cfg) {}

        [[nodiscard]] uint8_t address() const noexcept { return cfg.address; }
    };

    class bus_start_stop_block;

    class bus
    {
        bus_cfg cfg {};
        std::map<uint8_t, const device*> devices {};
    public:
        bus() = default;
        explicit bus(const bus_cfg& cfg) : cfg(cfg)
        {
            gpio_set_direction(sda(), GPIO_MODE_INPUT_OUTPUT);
            gpio_set_direction(scl(), GPIO_MODE_INPUT_OUTPUT);

            gpio_set_level(sda(), DIGITAL(true));
            gpio_set_level(scl(), DIGITAL(true));
        }
        bus(const bus_cfg& cfg, const device* handles[], const size_t handles_len) : bus(cfg) { register_devices(handles, handles_len); }
        bus(const bus_cfg& cfg, const device devs[], const size_t devs_len) : bus(cfg) { register_devices(devs, devs_len); }
        bus(const bus_cfg& cfg, device* handles[], const size_t handles_len) : bus(cfg) { register_devices(handles, handles_len); }
        bus(const bus_cfg& cfg, device devs[], const size_t devs_len) : bus(cfg) { register_devices(devs, devs_len); }

        [[nodiscard]] gpio_num_t sda() const noexcept { return cfg.sda; }
        [[nodiscard]] gpio_num_t scl() const noexcept { return cfg.scl; }
        [[nodiscard]] size_t count() const noexcept { return devices.size(); }

        void sda(const bool value) const { gpio_set_level(sda(), DIGITAL(value)); }
        void scl(const bool value) const { gpio_set_level(scl(), DIGITAL(value)); }
        void toggle_sda() const { gpio_set_level(sda(), INVERT_DIGITAL(gpio_get_level(sda()))); }
        void toggle_scl() const { gpio_set_level(scl(), INVERT_DIGITAL(gpio_get_level(scl()))); }

        [[nodiscard]] const device* get_device(const uint8_t address) const { return devices.at(address); }
        [[nodiscard]] const device* get_device(const dev_cfg& dev) const { return get_device(dev.address); }
        [[nodiscard]] const device* get_device(const device* handle) const { return get_device(handle->address()); }
        [[nodiscard]] std::optional<const device*> try_get_device (const uint8_t address) const noexcept
        {
            try
            {
                if (const auto iter = devices.find(address); iter != devices.end())
                    return iter->second;
            }
            catch (...) {}

            return std::nullopt;
        }
        [[nodiscard]] std::optional<const device*> try_get_device (const dev_cfg& dev) const { return try_get_device(dev.address); }
        [[nodiscard]] std::optional<const device*> try_get_device (const device* handle) const { return try_get_device(handle->address()); }
        [[nodiscard]] bool device_exists(const uint8_t address) const noexcept { return try_get_device(address).has_value(); }
        [[nodiscard]] bool device_exists(const dev_cfg& dev) const noexcept { return try_get_device(dev).has_value(); }
        [[nodiscard]] bool device_exists(const device* handle) const noexcept { return try_get_device(handle).has_value(); }

        [[nodiscard]] std::map<uint8_t, const device*>::iterator begin() noexcept { return devices.begin(); }
        [[nodiscard]] std::map<uint8_t, const device*>::iterator end() noexcept { return devices.end(); }
        [[nodiscard]] std::map<uint8_t, const device*>::const_iterator cbegin() const noexcept { return devices.cbegin(); }
        [[nodiscard]] std::map<uint8_t, const device*>::const_iterator cend() const noexcept { return devices.cend(); }

        bus* register_device(const device* handle)
        {
            if (device_exists(handle))
                throw std::logic_error(std::format("Device {:02X} already registered", handle->address()));

            devices[handle->address()] = handle;

            return this;
        }

        bus* register_devices(const device* handles[], const size_t handles_len)
        {
            for (auto i = 0; i < handles_len; i++)
                register_device(handles[i]);

            return this;
        }

        bus* register_devices(const device devs[], const size_t devs_len)
        {
            for (auto i = 0; i < devs_len; i++)
                register_device(&devs[i]);

            return this;
        }

        bus* register_device(device* handle)
        {
            if (device_exists(handle))
                throw std::logic_error(std::format("Device {:02X} already registered", handle->address()));

            devices[handle->address()] = handle;

            return this;
        }

        bus* register_devices(device* handles[], const size_t handles_len)
        {
            for (auto i = 0; i < handles_len; i++)
                register_device(handles[i]);

            return this;
        }

        bus* register_devices(device devs[], const size_t devs_len)
        {
            for (auto i = 0; i < devs_len; i++)
                register_device(&devs[i]);

            return this;
        }

        std::optional<const device*> remove_device(const uint8_t address) noexcept
        {
            const auto maybe_device = try_get_device(address);
            devices.erase(address);
            return maybe_device;
        }
        std::optional<const device*> remove_device(const dev_cfg& dev) noexcept { return remove_device(dev.address); }
        std::optional<const device*> remove_device(const device* handle) noexcept { return remove_device(handle->address()); }

        [[nodiscard]] result send_start_condition() const
        {
            //Both lines high
            sda(true);
            scl(true);

            //Data line low
            vTaskDelay(1 / portTICK_PERIOD_US);
            sda(false);
            vTaskDelay(1 / portTICK_PERIOD_US);

            //Tick
            toggle_scl();

            return result::ok();
        }

        [[nodiscard]] result send_stop_condition() const
        {
            //Set clock to high
            scl(true);

            //Data line high
            vTaskDelay(1 / portTICK_PERIOD_US);
            sda(true);
            vTaskDelay(2 / portTICK_PERIOD_US);

            return result::ok();
        }

        [[nodiscard]] result send_data(const i_message& message) const
        {
            size_t message_data_len = 0;
            auto message_data = message.to_message(message_data_len);

            if (message_data_len == 0) //Empty message
                return result::ok();

            //TODO

            return result::general_error("NOT IMPLEMENTED");
        }

        [[nodiscard]] result receive_ack() const
        {
            //TODO
            return result::general_error("NOT IMPLEMENTED");
        }

        [[nodiscard]] result receive_nack() const
        {
            //TODO
            return result::general_error("NOT IMPLEMENTED");
        }

        [[nodiscard]] result receive_data() const
        {
            //TODO
            return result::general_error("NOT IMPLEMENTED");
        }

        bus_start_stop_block start_stop_block() const;
    };

    class bus_start_stop_block
    {
        const bus* handle;
    public:
        bus_start_stop_block() = delete;
        bus_start_stop_block(const bus_start_stop_block& other) = delete;
        bus_start_stop_block(bus_start_stop_block&& other) = delete;
        bus_start_stop_block& operator=(const bus_start_stop_block& other) = delete;
        bus_start_stop_block& operator=(bus_start_stop_block&& other) = delete;

        explicit bus_start_stop_block(const bus* handle) : handle(handle)
        {
            assert(handle != nullptr);
            if (handle == nullptr) return;

            if (const auto start_result = handle->send_start_condition(); start_result.is_err())
                throw std::runtime_error(start_result.error_message_or_default());
        }

        ~bus_start_stop_block()
        {
            assert(handle != nullptr);
            if (handle == nullptr) return;

            auto dummy = handle->send_stop_condition();
        }
    };

    inline bus_start_stop_block bus::start_stop_block() const { return bus_start_stop_block(this); }

    class controller
    {
        std::map<int8_t, const bus*> buses {};

        [[nodiscard]] static int8_t sda_for(const gpio_num_t sda) noexcept { return static_cast<int8_t>(sda); }
        [[nodiscard]] static int8_t sda_for(const bus_cfg& cfg) noexcept { return static_cast<int8_t>(cfg.sda); }
        [[nodiscard]] static int8_t sda_for(const bus* handle) noexcept { return static_cast<int8_t>(handle->sda()); }
        [[nodiscard]] static int8_t sda_for(bus* handle) noexcept { return static_cast<int8_t>(handle->sda()); }
    public:
        controller() = default;
        controller(const bus* handles[], const size_t handles_len) { register_buses(handles, handles_len); }
        controller(const bus buses_to_register[], const size_t buses_len)  { register_buses(buses_to_register, buses_len); }

        [[nodiscard]] const bus* get_bus(const int8_t sda) const { return buses.at(sda); }
        [[nodiscard]] const bus* get_bus(const gpio_num_t sda) const { return get_bus(sda_for(sda)); }
        [[nodiscard]] const bus* get_bus(const bus_cfg& cfg) const { return get_bus(sda_for(cfg)); }
        [[nodiscard]] const bus* get_bus(const bus* handle) const { return get_bus(sda_for(handle)); }

        [[nodiscard]] std::optional<const bus*> try_get_bus(const int8_t sda) const noexcept
        {
            try
            {
                if (const auto iter = buses.find(sda); iter != buses.end())
                    return iter->second;
            }
            catch (...) {}

            return std::nullopt;
        }
        [[nodiscard]] std::optional<const bus*> try_get_bus(const gpio_num_t sda) const noexcept { return try_get_bus(sda_for(sda)); }
        [[nodiscard]] std::optional<const bus*> try_get_bus(const bus_cfg& cfg) const noexcept { return try_get_bus(sda_for(cfg)); }
        [[nodiscard]] std::optional<const bus*> try_get_bus(const bus* handle) const { return try_get_bus(sda_for(handle)); }

        [[nodiscard]] bool bus_exists(const int8_t sda) const noexcept { return try_get_bus(sda).has_value(); }
        [[nodiscard]] bool bus_exists(const gpio_num_t sda) const noexcept { return try_get_bus(sda).has_value(); }
        [[nodiscard]] bool bus_exists(const bus_cfg& cfg) const noexcept { return try_get_bus(cfg).has_value(); }
        [[nodiscard]] bool bus_exists(const bus* handle) const noexcept { return try_get_bus(handle).has_value(); }

        [[nodiscard]] std::map<int8_t, const bus*>::iterator begin() noexcept { return buses.begin(); }
        [[nodiscard]] std::map<int8_t, const bus*>::iterator end() noexcept { return buses.end(); }
        [[nodiscard]] std::map<int8_t, const bus*>::const_iterator cbegin() const noexcept { return buses.cbegin(); }
        [[nodiscard]] std::map<int8_t, const bus*>::const_iterator cend() const noexcept { return buses.cend(); }

        controller* register_bus(const bus* handle)
        {
            if (bus_exists(handle))
                throw std::logic_error(std::format("Bus sda={:02X} already registered", static_cast<int8_t>(handle->sda())));
            buses[handle->sda()] = handle;

            return this;
        }

        controller* register_buses(const bus* handles[], const size_t handles_len)
        {
            for (auto i = 0; i < handles_len; i++)
                register_bus(handles[i]);

            return this;
        }

        controller* register_buses(const bus register_buses[], const size_t buses_len)
        {
            for (auto i = 0; i < buses_len; i++)
                register_bus(&register_buses[i]);

            return this;
        }

        std::optional<const bus*> remove_bus(const int8_t sda) noexcept
        {
            const auto maybe_bus = try_get_bus(sda);
            buses.erase(sda);
            return maybe_bus;
        }
        std::optional<const bus*> remove_bus(const gpio_num_t sda) noexcept { return remove_bus(sda_for(sda)); }
        std::optional<const bus*> remove_bus(const bus_cfg& cfg) noexcept { return remove_bus(sda_for(cfg)); }
        std::optional<const bus*> remove_bus(const bus* handle) noexcept { return remove_bus(sda_for(handle)); }
    };
}

#endif //AUDIO_CONTROLLER_I2C_BUS_H
//
// Created by bendstein on 10/23/2026.
//

#ifndef AUDIO_CONTROLLER_UART_RECEIVE_QUEUE_H
#define AUDIO_CONTROLLER_UART_RECEIVE_QUEUE_H
#include <cstdint>
#include <cstddef>

#include <driver/uart.h>

class uart_receive_queue
{
    uint8_t* buf;
    uint8_t* buf2;
    size_t size;
    size_t size2;
    size_t message_size;
    size_t read_cursor;
    size_t write_cursor;
    uart_port_t port;
    uint8_t pattern;

    [[nodiscard]] size_t get_read_index(size_t cursor) const;
    [[nodiscard]] size_t get_write_index(size_t cursor) const;
    [[nodiscard]] size_t get_read_length() const;
    [[nodiscard]] size_t get_write_length() const;
public:
    uart_receive_queue(uart_port_t uart_port, uint8_t* buffer, size_t buffer_size, uint8_t* buffer2, size_t buffer2_size, size_t message_size, uint8_t pattern);

    esp_err_t fill_buffer(TickType_t timeout = portMAX_DELAY);
    esp_err_t next(uint8_t* out, size_t out_len, size_t& copied);
};

#endif //AUDIO_CONTROLLER_UART_RECEIVE_QUEUE_H
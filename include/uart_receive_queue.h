//
// Created by bendstein on 10/23/2026.
//

#ifndef AUDIO_CONTROLLER_UART_RECEIVE_QUEUE_H
#define AUDIO_CONTROLLER_UART_RECEIVE_QUEUE_H
#include <cstdint>
#include <cstddef>

#include <driver/uart.h>

/**
 * Queue to receive messages from UART
 */
class uart_receive_queue
{
    /**
     * Data buffer that is written to/read from
     */
    uint8_t* buf;

    /**
     * Temp buffer that is used when writing
     * to buf
     */
    uint8_t* buf2;

    /**
     * Size of buf, should be at least as big as message_size
     */
    size_t size;

    /**
     * Size of buf2
     */
    size_t size2;

    /**
     * Bytes per message
     */
    size_t message_size;

    /**
     * Read index into buf
     */
    size_t read_cursor;

    /**
     * Write index into buf
     */
    size_t write_cursor;

    /**
     * UART port
     */
    uart_port_t port;

    /**
     * Each message is prefixed by this,
     * repeated message_size times
     */
    uint8_t pattern;

    [[nodiscard]] size_t get_read_index(size_t cursor) const;
    [[nodiscard]] size_t get_write_index(size_t cursor) const;
    [[nodiscard]] size_t get_read_length() const;
    [[nodiscard]] size_t get_write_length() const;
public:
    /**
     * @param uart_port UART port
     * @param buffer Data buffer
     * @param buffer_size Size of buffer, should be at least as big as message_size
     * @param buffer2 Temp buffer used during write to buffer
     * @param buffer2_size Size of buffer2
     * @param message_size Bytes per message
     * @param pattern Message prefix, repeated message_size times
     */
    uart_receive_queue(uart_port_t uart_port, uint8_t* buffer, size_t buffer_size, uint8_t* buffer2, size_t buffer2_size, size_t message_size, uint8_t pattern);

    /**
     * Fill buf from uart buffer
     * @param timeout Timeout in ticks
     * @return ESP_OK if successful, else an error code
     */
    esp_err_t fill_buffer(TickType_t timeout = portMAX_DELAY);

    /**
     * Get bytes for next message
     * @param out Output data buffer
     * @param out_len Length of output data buffer (should be at least message_size)
     * @param copied Number of bytes written to out
     * @return ESP_OK if successful, else an error code
     */
    esp_err_t next(uint8_t* out, size_t out_len, size_t& copied);
};

#endif //AUDIO_CONTROLLER_UART_RECEIVE_QUEUE_H
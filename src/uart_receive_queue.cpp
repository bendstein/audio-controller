//
// Created by bendstein on 10/23/2026.
//
#include <algorithm>

#include "app_common.h"
#include "uart_receive_queue.h"

uart_receive_queue::uart_receive_queue(const uart_port_t uart_port, uint8_t* buffer, const size_t buffer_size, uint8_t* buffer2, const size_t buffer2_size, const size_t message_size, const uint8_t pattern)
    : buf(buffer), buf2(buffer2), size(buffer_size), size2(buffer2_size), message_size(message_size), read_cursor(0), write_cursor(0), port(uart_port), pattern(pattern)
{
    assert(buf != nullptr);
    assert(buf2 != nullptr);
    assert(message_size > 0);
    assert(buffer_size >= message_size);
    assert(buffer2_size > 0);
}

size_t uart_receive_queue::get_read_index(const size_t cursor) const
{
    if (cursor >= get_read_length())
        throw std::out_of_range("Read index out of range.");

    return (read_cursor + cursor) % size;
}

size_t uart_receive_queue::get_write_index(const size_t cursor) const
{
    if (cursor >= get_write_length())
        throw std::out_of_range("Write index out of range.");

    return (write_cursor + cursor) % size;
}

size_t uart_receive_queue::get_read_length() const
{
    if (write_cursor >= read_cursor)
        return write_cursor - read_cursor;
    return size - 1 - (read_cursor - write_cursor);
}

size_t uart_receive_queue::get_write_length() const
{
    return size - get_read_length();
}

esp_err_t uart_receive_queue::fill_buffer(const TickType_t timeout)
{
    //Get available space
    const auto max_len = std::min(get_write_length(), size2);

    if (max_len == 0) //No space
        return ESP_OK;

    //Get data length (cap at available space)
    size_t data_len = 0;

    if (const auto get_len_result = uart_get_buffered_data_len(port, &data_len);
        get_len_result != ESP_OK)
    {
        return get_len_result; //Get length error
    }

    data_len = std::min(data_len, max_len);

    if (data_len == 0) //No data
    {
        // VERBOSE("0 bytes added to uart receive queue (uart_get_buffered_data_len -> 0).");
        return ESP_OK;
    }

    //Read data
    const auto read_result = uart_read_bytes(port, buf2, data_len, timeout);

    if (read_result == 0) //No data
    {
        // VERBOSE("0 bytes added to uart receive queue (uart_read_bytes -> 0).");
        return ESP_OK;
    }

    if (read_result < 0) //Read error
        return read_result;

    data_len = read_result;

    //Populate buffer
    auto copy_from = buf2;

    //Populate from write_cursor to end of array
    if (write_cursor >= read_cursor)
    {
        const auto copy_len_0 = std::min(size - write_cursor, data_len);
        memcpy(buf + write_cursor, copy_from, copy_len_0);
        write_cursor = (write_cursor + copy_len_0) % size;
        copy_from += copy_len_0;
        data_len -= copy_len_0;
    }

    //If any remaining data, populate from write_cursor to read_cursor
    if (data_len > 0)
    {
        const auto copy_len_1 = std::min(get_write_length(), data_len);
        memcpy(buf + write_cursor, copy_from, copy_len_1);
        write_cursor = (write_cursor + copy_len_1) % size;
        data_len -= copy_len_1;
    }

    assert(data_len == 0);

    FVERBOSE("{} bytes added to uart receive queue. (r = {}, w = {})", read_result, read_cursor, write_cursor);

    return ESP_OK;
}

esp_err_t uart_receive_queue::next(uint8_t* out, const size_t out_len, size_t& copied)
{
    copied = 0;
    if (out == nullptr)
        return ESP_ERR_INVALID_ARG;
    if (out_len < message_size)
        return ESP_ERR_INVALID_ARG;

    if (get_read_length() < 2 * message_size)
        return ESP_OK; //Buffer isn't big enough contain a full message and start pattern

    //Search for start pattern ({pattern} x{message_size})
    bool seen_any_pattern_item = false;
    bool found_pattern = false;
    size_t pattern_start_index = 0;
    size_t message_start_index = 0;

    for (size_t i = 0; i < get_read_length(); i++)
    {
        if (const auto item = buf[get_read_index(i)]; item == pattern)
        {
            //Update pattern start index to be the most recent item of the pattern
            pattern_start_index = get_read_index(i);
            seen_any_pattern_item = true;

            if (message_size == 1)
            {
                found_pattern = true;
                message_start_index = get_read_index(i + 1);
            }
            else
            {
                for (size_t lah = 1; i + lah < get_read_length(); lah++)
                {
                    if (const auto lah_item = buf[get_read_index(i + lah)]; lah_item != pattern)
                        break;

                    if (lah + 1 == message_size)
                    {
                        found_pattern = true;
                        message_start_index = get_read_index(i + lah + 1);
                        break;
                    }
                }
            }
        }

        if (found_pattern)
            break;
    }

    //If seen the pattern char, advance to the most recent instance of it
    if (seen_any_pattern_item)
        read_cursor = pattern_start_index;

    if (!found_pattern)
    {
        if (!seen_any_pattern_item) //If the start pattern wasn't found, and no chars matched, clear read buffer
            read_cursor = write_cursor;

        VERBOSE("No start pattern.");
        return ESP_OK; //Start pattern is missing
    }

    if (get_read_length() < 2 * message_size)
        return ESP_OK; //Buffer isn't big enough contain a full message and start pattern

    //Update read cursor to be the start of the message
    read_cursor = message_start_index;

    uint8_t* copy_to = out;
    size_t copy_to_len = out_len - (out_len % message_size);
    size_t total_read = 0;

    //Copy from read cursor to end of array
    if (read_cursor > write_cursor)
    {
        const auto copy_len_0 = std::min(size - read_cursor, copy_to_len);
        memcpy(copy_to, buf + read_cursor, copy_len_0);
        read_cursor = (read_cursor + copy_len_0) % size;
        copy_to += copy_len_0;
        copy_to_len -= copy_len_0;
        total_read += copy_len_0;
    }

    //If any remaining data, copy from read cursor to write cursor
    if (copy_to_len > 0)
    {
        const auto copy_len_1 = std::min(get_read_length(), copy_to_len);
        memcpy(copy_to, buf + read_cursor, copy_len_1);
        read_cursor = (read_cursor + copy_len_1) % size;
        total_read += copy_len_1;
    }

    copied = total_read;
    return ESP_OK;
}
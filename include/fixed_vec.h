//
// Created by bendstein on 1/25/2026.
//

#ifndef AUDIO_CONTROLLER_FIXED_VEC_H
#define AUDIO_CONTROLLER_FIXED_VEC_H

#include <cstring>
#include <format>
#include <stdexcept>

template<typename T, size_t CAPACITY> requires (CAPACITY > 0)
class fixed_vec
{
public:
    static constexpr size_t capacity = CAPACITY;
private:
    size_t length;
    T storage[CAPACITY];
public:
    fixed_vec() : length(0), storage()
    {
        memset(storage, 0, CAPACITY * sizeof(T));
    }

    fixed_vec(const fixed_vec& other) = delete;
    fixed_vec(fixed_vec&& other) noexcept : length(0), storage()
    {
        clone_from(other);
    }

    ~fixed_vec() = default;

    fixed_vec& operator=(const fixed_vec& other)
    {
        if (this == &other)
            return *this;

        *this = other;
        return *this;
    }

    fixed_vec& operator=(fixed_vec&& other) noexcept
    {
        if (this == &other)
            return *this;

        clone_from(other);
        return *this;
    }

    T& operator[](const size_t index) { return at(index); }
    const T& operator[](const size_t index) const { return at(index); }
    T* operator*() { return storage; }

    [[nodiscard]] T* data() noexcept { return storage; }
    [[nodiscard]] const T* data() const noexcept { return storage; }
    [[nodiscard]] size_t size() const noexcept { return length; }
    [[nodiscard]] bool empty() const noexcept { return length == 0; }
    [[nodiscard]] bool full() const noexcept { return length == CAPACITY; }

    [[nodiscard]] bool sequence_equals(const fixed_vec& other) const
    {
        if (this == &other)
            return true;

        if (length != other.length)
            return false;

        for (size_t i = 0; i < length; i++)
        {
            if (storage[i] != other.storage[i])
                return false;
        }

        return true;
    }

    [[nodiscard]] const T& at(size_t i) const
    {
        if (i >= length)
            throw std::runtime_error(std::format("Index {} out of range (length = {})", i, length));

        return storage[i];
    }
    [[nodiscard]] const T& front() const { return at(0); }
    [[nodiscard]] const T& back() const { return at(length - 1); }
    [[nodiscard]] T& at(size_t i)     {
        if (i >= length)
            throw std::runtime_error(std::format("Index {} out of range (length = {})", i, length));

        return storage[i];
    }
    [[nodiscard]] T& front() { return at(0); }
    [[nodiscard]] T& back() { return at(length - 1); }

    void clone_from(const fixed_vec& other) noexcept
    {
        length = other.length;
        memcpy(storage, other.storage, sizeof(T) * CAPACITY);
    }

    void clear()
    {
        length = 0;
        memset(storage, 0, CAPACITY * sizeof(T));
    }

    bool remove_last()
    {
        if (length == 0)
            return false;

        length--;

        return true;
    }

    size_t add_to_end(T value)
    {
        if (length == CAPACITY)
            throw std::runtime_error(std::format("Cannot add past capacity ({})", CAPACITY));

        size_t index = length++;
        storage[index] = value;

        return index;
    }

    size_t grow()
    {
        if (length == CAPACITY)
            throw std::runtime_error(std::format("Cannot add past capacity ({})", CAPACITY));

        return length++;
    }

    void replace_at(T value, size_t i, T& previous_value)
    {
        if (i >= length)
            throw std::runtime_error(std::format("Index {} out of range (length = {})", i, length));

        previous_value = storage[i];
        storage[i] = value;
    }
};

#endif //AUDIO_CONTROLLER_FIXED_VEC_H
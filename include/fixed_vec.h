//
// Created by bendstein on 1/25/2026.
//

#ifndef AUDIO_CONTROLLER_FIXED_VEC_H
#define AUDIO_CONTROLLER_FIXED_VEC_H

#include <format>
#include <stdexcept>

template<typename T, size_t CAPACITY> requires (CAPACITY > 0)
class fixed_vec
{
public:
    static constexpr size_t capacity = CAPACITY;
private:
    size_t length = 0;
    T storage[CAPACITY] {};
public:
    fixed_vec()
    {
        for (auto i = 0; i < CAPACITY; i++)
            storage[i] = nullptr;
    }

    explicit fixed_vec(T pre_allocated[CAPACITY]) : storage(pre_allocated)
    {
        for (auto i = 0; i < CAPACITY; i++)
            storage[i] = nullptr;
    }

    T& operator[](const size_t index) { return at(index); }
    const T& operator[](const size_t index) const { return at(index); }

    [[nodiscard]] T* data() noexcept { return storage; }
    [[nodiscard]] const T* data() const noexcept { return storage; }
    [[nodiscard]] size_t size() const noexcept { return length; }
    [[nodiscard]] bool empty() const noexcept { return length == 0; }
    [[nodiscard]] bool full() const noexcept { return length == CAPACITY; }

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

    bool remove_last()
    {
        if (length == 0)
            return false;

        storage[length - 1] = nullptr;
        length--;

        return true;
    }

    void remove_at(size_t i)
    {
        if (i >= length)
            throw std::runtime_error(std::format("Index {} out of range (length = {})", i, length));

        auto current = at(i);
        storage[i] = nullptr;

        if (i == length - 1) //Reduce length if removing from end
            length--;
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
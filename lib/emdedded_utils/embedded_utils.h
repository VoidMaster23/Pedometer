#pragma once
#include <cstdint>
#include <concepts>

namespace Utils
{
    template <std::integral T>
    struct EMA_filter
    {
        T accumulator{0};
        const int8_t EMA_ALPHA{2};

        T ema_filter(T value)
        {
            T average;
            accumulator += value;
            average = (accumulator - (accumulator < 0) + (1 << (EMA_ALPHA - 1))) >> EMA_ALPHA;
            accumulator -= average;
            return value - average;
        }
    };
}
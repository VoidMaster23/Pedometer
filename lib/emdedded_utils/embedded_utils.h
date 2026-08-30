#pragma once
#include <cstdint>
#include <concepts>
#include <array>
#include <cmath>
#include <algorithm>

namespace Utils
{
    static constexpr int16_t SENSITIVITY{350};

    template <std::integral T>
    struct SamplePoint
    {
        std::size_t index = 0;
        T value = 0;
    };

    template <std::integral T>
    struct MinMaxResult
    {
        SamplePoint<T> min;
        SamplePoint<T> max;
    };

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

    template <std::integral T, std::size_t N>
        requires(N > 0)
    class CircularBuffer
    {
    private:
        std::array<T, N> buffer{};
        std::size_t index = 0;

    public:
        CircularBuffer() = default;

        explicit constexpr CircularBuffer(T fill_value)
        {
            buffer.fill(fill_value);
        }

        void push(T val)
        {
            buffer[index] = val;
        }

        void advance_head()
        {
            index = (index + 1) % N;
        }

        [[nodiscard]] constexpr const T &head_element() const noexcept
        {
            return buffer[index];
        }

        [[nodiscard]] constexpr const T &operator[](std::size_t ind) const noexcept
        {
            return buffer[ind];
        }

        [[nodiscard]] constexpr auto begin() const noexcept
        {
            return buffer.begin();
        }

        [[nodiscard]] constexpr auto end() const noexcept
        {
            return buffer.end();
        }

        T circular_delta(T target_index)
        {
            T target_center = ((index + (N >> 1)) % N);

            // Safely calculate the absolute difference without using std::abs
            T diff = (target_index > target_center) ? (target_index - target_center)
                                                    : (target_center - target_index);

            return (std::min)(diff, static_cast<T>(N) - diff);
        }

        [[nodiscard]] constexpr std::size_t circular_distance(std::size_t a, std::size_t b) const noexcept
        {
            const auto direct_diff = static_cast<std::size_t>(
                std::abs(static_cast<std::ptrdiff_t>(a) - static_cast<std::ptrdiff_t>(b)));
            return std::min(direct_diff, N - direct_diff);
        }
        /* data */
    };

    struct ThresholdState
    {

    private:
        static constexpr int16_t INITIAL_OFFSET{4096}; // offset by 1g (might not be needed but hey again fuck it we ball)
        static constexpr int8_t THRESHOLD_ORDER{4};
        Utils::CircularBuffer<int32_t, THRESHOLD_ORDER> dynamic_threshold_buffer{INITIAL_OFFSET};

    public:
        int32_t dynamic_threshold{INITIAL_OFFSET * THRESHOLD_ORDER};
        int32_t old_threshold{INITIAL_OFFSET};
        int32_t new_threshold{0};
        int32_t difference_between_max_min{0};

        int8_t threshold_count{0};

        [[nodiscard]] constexpr bool update(int32_t max, int32_t min)
        {
            difference_between_max_min = max - min;
            if (difference_between_max_min > SENSITIVITY)
            {
                new_threshold = (max + min) >> 1;
                dynamic_threshold = dynamic_threshold - dynamic_threshold_buffer.head_element() + new_threshold;
                old_threshold = dynamic_threshold / THRESHOLD_ORDER;
                dynamic_threshold_buffer.push(new_threshold);
                dynamic_threshold_buffer.advance_head();
                return true;
            }
            return false;
        }
    };

}
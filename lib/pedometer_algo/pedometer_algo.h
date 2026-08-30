#pragma once
#include <cstdint>
#include "embedded_utils.h"
#include <array>
#include <tuple>

class PedometerAlgo
{
public:
    enum class State : uint8_t
    {
        Idle,
        WaitingForMin
    };

    enum class TickEvent : uint8_t
    {
        None,
        CycleComplete,
        Timeout
    };

private:
    static constexpr int8_t FILTER_ORDER{9};

    static constexpr int8_t WINDOW_SIZE{(FILTER_ORDER << 2) + 1};

    static constexpr int8_t ONE_SEC{100}; // the number of samples we will be considering in a second
    static constexpr int16_t REGULATION_MODE_OFF_TIMEOUT{(ONE_SEC << 1)};

    static constexpr int8_t THRESHOLD_STEP_COUNT_FOR_REGULATION{6};

    Utils::EMA_filter<int32_t> accumulator_x;
    Utils::EMA_filter<int32_t> accumulator_y;
    Utils::EMA_filter<int32_t> accumulator_z;

    Utils::CircularBuffer<int32_t, FILTER_ORDER> raw_data{};
    Utils::CircularBuffer<int32_t, WINDOW_SIZE> filtered_window{};

    void add_data_to_buffers(int16_t x, int16_t y, int16_t z);

    Utils::MinMaxResult<int32_t> get_max_min_window_indices();

    State state = State::Idle;
    int16_t samples_since_max{0};

    int32_t last_max_value{0};
    int32_t last_min_value{0};

    int8_t possible_steps{0};
    int32_t step_count{0};
    int32_t algo_iterations{0};

    bool is_regulation_mode_active;

    Utils::ThresholdState threshold;

    [[nodiscard]] constexpr bool is_valid_amplitude(const int32_t peak, const int32_t valley) const noexcept;

    // void determine_possible_step()

public:
    PedometerAlgo() = default;

    constexpr TickEvent process_tick(bool is_max_centered, bool is_min_centered, int32_t max, int32_t min)
    {
        switch (state)
        {
        case State::Idle:
            if (is_max_centered)
            {
                last_max_value = max;
                samples_since_max = 0;
                state = State::WaitingForMin;
            }
            return TickEvent::None;

        case State::WaitingForMin:
            samples_since_max++;
            if (is_min_centered)
            {
                last_min_value = min;
                state = State::Idle;
                samples_since_max = 0;
                return TickEvent::CycleComplete;
            }
            else if (samples_since_max >= ONE_SEC)
            {
                state = State::Idle;
                samples_since_max = 0;
                return TickEvent::Timeout;
            }
            return TickEvent::None;

        default:
            return TickEvent::None;
        }
    }

    [[nodiscard]] constexpr State current_state() const noexcept
    {
        return state;
    }

    [[nodiscard]] constexpr bool is_waiting_for_min() const noexcept
    {
        return state == State::WaitingForMin;
    }

    void reset_counts() noexcept
    {
        algo_iterations = 0;
        possible_steps = 0;
        is_regulation_mode_active = false;
        // threshold = Utils::ThresholdState{};
    }

    int32_t count_steps(int16_t x, int16_t y, int16_t z);
};

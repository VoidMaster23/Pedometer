#include "pedometer_algo.h"
#include <cmath>

#include <algorithm>
#include <array>
#include <ranges>
#include "embedded_utils.h"

#include <Arduino.h>

void PedometerAlgo::add_data_to_buffers(int16_t x, int16_t y, int16_t z)
{
    int32_t fltered_x = accumulator_x.ema_filter(x);
    int32_t fltered_y = accumulator_y.ema_filter(y);
    int32_t fltered_z = accumulator_z.ema_filter(z);

    int32_t input_data = std::abs(fltered_x) + std::abs(fltered_y) + std::abs(fltered_z);

    std::array<int32_t, FILTER_ORDER> median_data{};
    std::ranges::copy(raw_data, std::begin(median_data));
    std::ranges::nth_element(median_data, median_data.begin() + median_data.size() / 2);

    raw_data.push(input_data);
    filtered_window.push(median_data[median_data.size() / 2]);
}

Utils::MinMaxResult<int32_t> PedometerAlgo::get_max_min_window_indices()
{
    const auto [min, max] = std::ranges::minmax_element(filtered_window);

    const auto min_index = std::distance(filtered_window.begin(), min);
    const auto max_index = std::distance(filtered_window.begin(), max);

    Utils::MinMaxResult<int32_t> result{};

    result.max.value = *max;
    result.max.index = max_index;

    result.min.value = *min;
    result.min.index = min_index;

    return result;
}

constexpr bool PedometerAlgo::is_valid_amplitude(const int32_t peak, const int32_t valley) const noexcept
{
    return peak > (threshold.old_threshold + (Utils::SENSITIVITY >> 1)) && ((valley + (Utils::SENSITIVITY >> 1)) < threshold.old_threshold);
}

int32_t PedometerAlgo::count_steps(int16_t x, int16_t y, int16_t z)
{
    add_data_to_buffers(x, y, z);
    auto [min, max] = get_max_min_window_indices();

    bool is_max_in_middle = filtered_window.circular_delta(max.index) < 2;
    bool is_min_in_middle = filtered_window.circular_delta(min.index) < 2;

    TickEvent tick_result = process_tick(is_max_in_middle, is_min_in_middle, max.value, min.value);

    switch (tick_result)
    {
    case TickEvent::CycleComplete:
        // Serial.print(max.value);
        // Serial.print(" ");
        // Serial.println(min.value);
        algo_iterations = 0;

                if (is_valid_amplitude(last_max_value, last_min_value))
        {
            threshold.threshold_count = 0;
            threshold.update(last_max_value, last_min_value);

            if (is_regulation_mode_active)
            {
                step_count++;
            }
            else
            {
                possible_steps++;
                Serial.println(possible_steps);
                if (possible_steps >= THRESHOLD_STEP_COUNT_FOR_REGULATION)
                {
                    Serial.println("HERE");
                    step_count += possible_steps;
                    possible_steps = 0;
                    is_regulation_mode_active = true;
                }
            }
        }
        else
        {
            threshold.threshold_count++;
            if (threshold.threshold_count > 1)
            {
                threshold.threshold_count = 0;
                possible_steps = 0;
                is_regulation_mode_active = false;
            }
        }
        break;
    case TickEvent::Timeout:
        possible_steps = 0;
        break;

    default:
        break;
    }

    filtered_window.advance_head();
    raw_data.advance_head();

    algo_iterations++;
    if (algo_iterations >= REGULATION_MODE_OFF_TIMEOUT)
    {
        reset_counts();
    }

    return step_count;
}
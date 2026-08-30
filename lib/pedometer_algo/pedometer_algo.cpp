#include "pedometer_algo.h"
#include <cmath>
#include <algorithm>
#include <array>
#include <ranges>
#include "embedded_utils.h"

auto PedometerAlgo::add_data_to_buffers(int16_t accel_x, int16_t accel_y, int16_t accel_z) -> void
{
    int32_t filtered_x{accumulator_x.ema_filter(accel_x)};
    int32_t filtered_y{accumulator_y.ema_filter(accel_y)};
    int32_t filtered_z{accumulator_z.ema_filter(accel_z)};

    int32_t input_data{std::abs(filtered_x) + std::abs(filtered_y) + std::abs(filtered_z)};

    std::array<int32_t, FILTER_ORDER> median_data{};
    std::ranges::copy(raw_data, std::begin(median_data));
    std::ranges::nth_element(median_data, median_data.begin() + median_data.size() / 2);

    raw_data.push(input_data);
    filtered_window.push(median_data[median_data.size() / 2]);
}

auto PedometerAlgo::get_max_min_window_indices() -> Utils::MinMaxResult<int32_t>
{
    const auto min = std::ranges::min_element(filtered_window);
    const auto max = std::ranges::max_element(filtered_window);

    const auto min_index = std::distance(filtered_window.begin(), min);
    const auto max_index = std::distance(filtered_window.begin(), max);

    Utils::MinMaxResult<int32_t> result{};

    result.max.value = *max;
    result.max.index = static_cast<std::size_t>(max_index);

    result.min.value = *min;
    result.min.index = static_cast<std::size_t>(min_index);

    return result;
}

auto PedometerAlgo::count_steps(int16_t accel_x, int16_t accel_y, int16_t accel_z) -> int32_t
{
    add_data_to_buffers(accel_x, accel_y, accel_z);
    auto [min, max] = get_max_min_window_indices();

    if ((max.value - min.value) > Utils::SENSITIVITY)
    {
        (void)threshold.update(max.value, min.value);
    }

    bool is_max_in_middle{filtered_window.circular_delta(max.index) < 2};
    bool is_min_in_middle{filtered_window.circular_delta(min.index) < 2};

    TickEvent tick_result = process_tick(is_max_in_middle, is_min_in_middle, max.value, min.value);

    switch (tick_result)
    {
    case TickEvent::CycleComplete:
        if ((last_max_value - last_min_value) > Utils::SENSITIVITY)
        {
            algo_iterations = 0;
        }

        if (is_valid_amplitude(last_max_value, last_min_value))
        {
            threshold.threshold_count = 0;

            if (is_regulation_mode_active)
            {
                step_count++;
            }
            else
            {
                possible_steps++;
                if (possible_steps >= THRESHOLD_STEP_COUNT_FOR_REGULATION)
                {
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

#include "pedometer_algo.h"
#include <cmath>

#include <algorithm>
#include <array>
#include <ranges>

#define FILTER_ORDER 9
#define THRESHOLD_ORDER 4
#define WINDOW_SIZE (FILTER_ORDER << 2) + 1
#define SENSITIVITY 410     // this is 0.1g on an mpu 6050
#define INITIAL_OFFSET 4096 // offset by 1g (might not be needed but hey again fuck it we ball)
#define MIN_INITIAL_VALUE 0
#define ONE_SEC 100 // the number of samples we will be considering in a second
#define REGULATION_MODE_OFF_TIMEOUT (ONE_SEC << 1)

#define THRESHOLD_STEP_COUNT_FOR_REGULATION 6

constexpr int8_t EMA_ALPHA = 2;

int32_t accumulator_x;
int32_t accumulator_y;
int32_t accumulator_z;

// Globals (for now, some will be moved into the actual count steps function as I see fit)
int32_t raw_data[FILTER_ORDER];
int32_t filtered_window[WINDOW_SIZE];
int32_t dynamic_threshold_buffer[THRESHOLD_ORDER];
int32_t step_count; // yeah this is our algo output.. yay!

// these two can for sure be defined inside relevant function/helpers
int8_t min_window_index;
int8_t max_window_index;

// we track the index in the buffer, threshold and average
int8_t next_index_in_buffer;
int8_t next_index_in_threshold;
int8_t next_index_in_average;

bool is_regulation_mode_active;

int8_t algo_iterations;

int8_t possible_steps;

int8_t threshold_count;

int32_t last_max_value;
int32_t last_min_value;

int32_t window_max_value;
int32_t window_min_value;

int8_t samples_between_max_min;

int32_t filtered_mean;
int32_t filtered_data;

int32_t difference_between_max_min;

int32_t dynamic_threshold;
int32_t new_threshold;
int32_t old_threshold;

bool is_max_in_middle_of_window;
bool threshold_reached;

void PedometerAlgo::initGlobals()
{
    for (uint8_t i = 0; i < FILTER_ORDER; i++)
    {
        raw_data[i] = 0;
    }

    for (uint8_t i = 0; i < WINDOW_SIZE; i++)
    {
        filtered_window[i] = 0;
    }

    for (uint8_t i = 0; i < THRESHOLD_ORDER; i++)
    {
        dynamic_threshold_buffer[i] = INITIAL_OFFSET;
    }

    min_window_index = 0;
    max_window_index = 0;
    window_max_value = 0;
    window_min_value = 0;

    accumulator_x = 0;
    accumulator_y = 0;
    accumulator_z = 0;

    next_index_in_average = 0;
    next_index_in_buffer = 0;
    next_index_in_threshold = 0;

    is_regulation_mode_active = false;
    is_max_in_middle_of_window = false;
    threshold_reached = false;

    algo_iterations = 0;

    possible_steps = 0;
    threshold_count = 0;

    last_max_value = 0;
    last_min_value = 0;

    filtered_mean = 0;
    filtered_data = 0;

    difference_between_max_min = 0;
    samples_between_max_min = 0;

    dynamic_threshold = INITIAL_OFFSET * THRESHOLD_ORDER;
    new_threshold = 0;
    old_threshold = INITIAL_OFFSET;
    step_count = 0;
}

int16_t ema_filter(int16_t value, int32_t &accumulator)
{
    int16_t average;
    accumulator += value;
    average = (accumulator - (accumulator < 0) + (1 << (EMA_ALPHA - 1))) >> EMA_ALPHA;
    accumulator -= average;
    return value - average;
}

void add_data_to_buffers(int16_t x, int16_t y, int16_t z)
{
    int16_t fltered_x = ema_filter(x, accumulator_x);
    int16_t fltered_y = ema_filter(y, accumulator_y);
    int16_t fltered_z = ema_filter(z, accumulator_z);

    int32_t input_data = abs(fltered_x) + abs(fltered_y) + abs(fltered_z);
    // filtered_data = filtered_data + input_data - raw_data[next_index_in_average];
    // filtered_mean = filtered_data / FILTER_ORDER;
    std::array<uint32_t, FILTER_ORDER> median_data{};
    std::ranges::copy(std::begin(raw_data), std::end(raw_data), std::begin(median_data));
    std::ranges::nth_element(median_data, median_data.begin() + median_data.size() / 2);
    raw_data[next_index_in_average] = input_data;
    filtered_window[next_index_in_buffer] = median_data[median_data.size() / 2];
}


void detect_max_min()
{
    window_max_value = filtered_window[0];
    max_window_index = 0;
    window_min_value = filtered_window[0];
    min_window_index = 0;

    for (int8_t i = 0; i < WINDOW_SIZE; i++)
    {
        if (filtered_window[i] > window_max_value)
        {
            window_max_value = filtered_window[i];
            max_window_index = i;
        }

        if (filtered_window[i] < window_min_value)
        {
            window_min_value = filtered_window[i];
            min_window_index = i;
        }
    }
}

int8_t get_circular_delta(int8_t current_index, int8_t target_center, int8_t buffer_size)
{
    int8_t min = std::min(abs(current_index - target_center), buffer_size - abs(current_index - target_center));
    return min;
}
void check_if_max_in_middle()
{
    int8_t target_center = ((next_index_in_buffer + (WINDOW_SIZE >> 1)) % WINDOW_SIZE);
    int8_t delta = get_circular_delta(max_window_index, target_center, WINDOW_SIZE);

    if (delta < 2)
    {
        is_max_in_middle_of_window = true;
        last_max_value = window_max_value;
        samples_between_max_min = 0;
    }
}

[[nodiscard]] bool check_if_min_in_middle()
{
    int8_t target_center = ((next_index_in_buffer + (WINDOW_SIZE >> 1)) % WINDOW_SIZE);
    int8_t delta = get_circular_delta(min_window_index, target_center, WINDOW_SIZE);

    if (delta < 2)
    {
        last_min_value = window_min_value;
        difference_between_max_min = last_max_value - last_min_value;
        is_max_in_middle_of_window = false;
        samples_between_max_min = 0;
        return true;
    }

    samples_between_max_min++;
    if (samples_between_max_min >= ONE_SEC)
    {
        samples_between_max_min = 0;
        window_max_value = 0;
        window_min_value = 0;
        is_max_in_middle_of_window = false;
        possible_steps = 0;
    }

    return false;
}

void determine_possible_step()
{
    if (last_max_value > (old_threshold + (SENSITIVITY >> 1)) && ((last_min_value + (SENSITIVITY >> 1)) < old_threshold))
    {
        threshold_reached = true;
        threshold_count = 0;
    }
    else
    {
        threshold_count++;
    }
}

void validate_step()
{
    if (threshold_reached)
    {
        threshold_reached = false;
        samples_between_max_min = 0;

        if (is_regulation_mode_active)
        {
            step_count++;
        }
        else
        {
            possible_steps++;

            if (possible_steps == THRESHOLD_STEP_COUNT_FOR_REGULATION)
            {
                step_count += possible_steps;
                possible_steps = 0;
                is_regulation_mode_active = true;
            }
        }
    }
}

void update_threshold()
{
    if (difference_between_max_min > SENSITIVITY)
    {
        new_threshold = (last_max_value + last_min_value) >> 1;
        dynamic_threshold = dynamic_threshold - dynamic_threshold_buffer[next_index_in_threshold] + new_threshold;
        old_threshold = dynamic_threshold / THRESHOLD_ORDER;
        dynamic_threshold_buffer[next_index_in_threshold] = new_threshold;
        next_index_in_threshold = (next_index_in_threshold + 1) % THRESHOLD_ORDER;
        algo_iterations = 0;
        validate_step();
    }
}

void update_indices()
{
    next_index_in_buffer = (next_index_in_buffer + 1) % WINDOW_SIZE;
    next_index_in_average = (next_index_in_average + 1) % FILTER_ORDER;

    algo_iterations++;
}

void reset_counts()
{
    if (is_regulation_mode_active)
    {
        old_threshold = INITIAL_OFFSET;
        dynamic_threshold = INITIAL_OFFSET * THRESHOLD_ORDER;
        next_index_in_threshold = 0;
        threshold_count = 0;
        for (int8_t i = 0; i < THRESHOLD_ORDER; i++)
        {
            dynamic_threshold_buffer[i] = 0;
        }
    }
    algo_iterations = 0;
    possible_steps = 0;
    is_regulation_mode_active = false;
    samples_between_max_min = 0;
}

void check_sens()
{
    if (threshold_count > 1)
    {
        threshold_count = 0;
        samples_between_max_min = 0;
        window_max_value = 0;
        window_min_value = 0;
        is_regulation_mode_active = false;
        possible_steps = 0;
    }
}

int32_t PedometerAlgo::count_steps(int16_t x, int16_t y, int16_t z)
{
    add_data_to_buffers(x, y, z);
    detect_max_min();

    if (!is_max_in_middle_of_window)
    {
        check_if_max_in_middle();
    }
    else
    {
        if (check_if_min_in_middle())
        {
            determine_possible_step();
            update_threshold();
            check_sens();
        }
    }
    update_indices();

    if (algo_iterations >= REGULATION_MODE_OFF_TIMEOUT)
    {
        reset_counts();
    }

    return step_count;
}
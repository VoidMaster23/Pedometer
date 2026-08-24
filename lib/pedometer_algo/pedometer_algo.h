#pragma once
#include <cstdint>

namespace PedometerAlgo {
void initGlobals();
int32_t count_steps(int16_t x, int16_t y, int16_t z);
}
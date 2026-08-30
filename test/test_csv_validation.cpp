#include <gtest/gtest.h>
#include "pedometer_algo.h"
#include <fstream>
#include <string>
#include <sstream>

TEST(PedometerCSVTest, ValidateDatasetAccuracy) {
    std::ifstream file("test/data/normalised_data.csv");
    ASSERT_TRUE(file.is_open()) << "Failed to open dataset file";

    std::string line;
    std::getline(file, line); // Skip header

    PedometerAlgo::initGlobals();

    int32_t step_count = 0;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str, z_str;
        
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        std::getline(ss, z_str, ',');

        if (x_str.empty() || y_str.empty() || z_str.empty()) continue;

        int16_t x = static_cast<int16_t>(std::stoi(x_str));
        int16_t y = static_cast<int16_t>(std::stoi(y_str));
        int16_t z = static_cast<int16_t>(std::stoi(z_str));

        step_count = PedometerAlgo::count_steps(x, y, z);
    }

    // Baseline: Expected 867
    EXPECT_EQ(step_count, 867) << "Measured steps did not match expected 867. Actual was " << step_count;
}

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>

#include "thunderloop.h"

TEST(TestBattery, is_power_stable)
{
    std::ofstream file("/tmp/battery_test_1.log");
    file << "this is some placeholder text!";
    file.close();

    std::ifstream input_file("/tmp/battery_test_1.log");

    EXPECT_TRUE(isPowerStable(input_file));
    std::remove("/tmp/battery_test_1.log");
}

TEST(TestBattery, is_power_not_stable)
{
    std::ofstream output_file("/tmp/battery_test_2.log");
    output_file << "soctherm: OC ALARM 0x00000001";
    output_file.close();

    std::ifstream input_file("/tmp/battery_test_2.log");

    EXPECT_FALSE(isPowerStable(input_file));
    std::remove("/tmp/battery_test_2.log");
}

TEST(TestBattery, can_open_file)
{
    std::ifstream file("/var/log/dmesg");
    EXPECT_TRUE(file.is_open());
}

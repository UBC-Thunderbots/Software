#include <gtest/gtest.h>

#include "thunderloop.h"

TEST(TestBattery, is_power_stable)
{
    std::ifstream file("/var/log/dmesg");
    EXPECT_TRUE(isPowerStable(file));
}

TEST(TestBattery, can_open_file)
{
    std::ifstream file("/var/log/dmesg");
    EXPECT_TRUE(file.is_open());
}

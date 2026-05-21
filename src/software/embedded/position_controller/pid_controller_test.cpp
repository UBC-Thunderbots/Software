#include "software/embedded/position_controller/pid_controller.h"

#include <gtest/gtest.h>

TEST(PidControllerTest, OnlyProportionTermNonZero)
{
    PidController pid{1.0, 0.0, 0.0, 0.0};

    EXPECT_DOUBLE_EQ(pid.step(0.0), 0.0);
    EXPECT_DOUBLE_EQ(pid.step(0.0), 0.0);
    EXPECT_DOUBLE_EQ(pid.step(1.0), 1.0);
    EXPECT_DOUBLE_EQ(pid.step(8.0), 8.0);
    EXPECT_DOUBLE_EQ(pid.step(-5.0), -5.0);
    EXPECT_DOUBLE_EQ(pid.step(-1.0), -1.0);

    pid = PidController{5.0, 0.0, 0.0, 0.0};

    EXPECT_DOUBLE_EQ(pid.step(0.0), 0.0);
    EXPECT_DOUBLE_EQ(pid.step(0.0), 0.0);
    EXPECT_DOUBLE_EQ(pid.step(1.0), 5.0);
    EXPECT_DOUBLE_EQ(pid.step(8.0), 40.0);
    EXPECT_DOUBLE_EQ(pid.step(-5.0), -25.0);
    EXPECT_DOUBLE_EQ(pid.step(-1.0), -5.0);
}

TEST(PidControllerTest, OnlyIntegralTermNonZero)
{
    constexpr double k_i = 2.0;
    PidController pid{0.0, k_i, 0.0, 10.0};

    EXPECT_DOUBLE_EQ(pid.step(1.0), k_i * 1.0);
    EXPECT_DOUBLE_EQ(pid.step(1.0), k_i * 2.0);
    EXPECT_DOUBLE_EQ(pid.step(1.0), k_i * 3.0);
    EXPECT_DOUBLE_EQ(pid.step(0.5), k_i * 3.5);

    // switch error direction, integral term should reset
    EXPECT_DOUBLE_EQ(pid.step(-0.2), k_i * -0.2);
    EXPECT_DOUBLE_EQ(pid.step(-1.0), k_i * -1.2);
    EXPECT_DOUBLE_EQ(pid.step(0.0), k_i * 0.0);

    // should not accumulate integral term above max_integral
    EXPECT_DOUBLE_EQ(pid.step(9.0), k_i * 9.0);
    EXPECT_DOUBLE_EQ(pid.step(5.0), k_i * 10.0);
    EXPECT_DOUBLE_EQ(pid.step(1.0), k_i * 10.0);
}

TEST(PidControllerTest, OnlyDerivativeTermNonZero)
{
    constexpr double k_d = 3.0;
    PidController pid{0.0, 0.0, k_d, 0.0};

    EXPECT_DOUBLE_EQ(pid.step(0.0), k_d * 0.0);
    EXPECT_DOUBLE_EQ(pid.step(0.0), k_d * 0.0);
    EXPECT_DOUBLE_EQ(pid.step(1.0), k_d * 1.0);
    EXPECT_DOUBLE_EQ(pid.step(8.0), k_d * 7.0);
    EXPECT_DOUBLE_EQ(pid.step(-5.0), k_d * -13.0);
    EXPECT_DOUBLE_EQ(pid.step(0.0), k_d * 5.0);
    EXPECT_DOUBLE_EQ(pid.step(5.0, 0.5), k_d * 5.0 / 0.5);
    EXPECT_DOUBLE_EQ(pid.step(8.0, 2.0), k_d * 3.0 / 2.0);
}

TEST(PidControllerTest, GeneralApplication)
{
    constexpr double k_p          = 10.0;
    constexpr double k_i          = 3.0;
    constexpr double k_d          = -1.0;
    constexpr double max_integral = 10.0;
    PidController pid{k_p, k_i, k_d, max_integral};

    EXPECT_DOUBLE_EQ(pid.step(12.0, 0.75), k_p * 12.0 + k_i * 9.0 + k_d * 0.0);
    EXPECT_DOUBLE_EQ(pid.step(24.0, 0.75), k_p * 24.0 + k_i * 10.0 + k_d * 12.0 / 0.75);
    EXPECT_DOUBLE_EQ(pid.step(4.0), k_p * 4.0 + k_i * 10.0 + k_d * -20.0);
    EXPECT_DOUBLE_EQ(pid.step(0.0), k_p * 0.0 + k_i * 0.0 + k_d * -4.0);
    EXPECT_DOUBLE_EQ(pid.step(2.0), k_p * 2.0 + k_i * 2.0 + k_d * 2.0);
    EXPECT_DOUBLE_EQ(pid.step(-2.0), k_p * -2.0 + k_i * -2.0 + k_d * -4.0);
}

TEST(PidControllerTest, InvalidArgumentsToConstructor)
{
    EXPECT_NO_THROW(PidController(0.0, 0.0, 0.0, 9.0));
    EXPECT_NO_THROW(PidController(0.0, 0.0, 0.0, 1.0));
    EXPECT_NO_THROW(PidController(0.0, 0.0, 0.0, 0.0));
    EXPECT_THROW(PidController(0.0, 0.0, 0.0, -0.5), std::invalid_argument);
    EXPECT_THROW(PidController(0.0, 0.0, 0.0, -1.0), std::invalid_argument);
    EXPECT_THROW(PidController(0.0, 0.0, 0.0, -3.0), std::invalid_argument);
}

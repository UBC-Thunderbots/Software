#include "software/embedded/motion_control/pid_controller.h"

#include <gtest/gtest.h>

TEST(PidControllerTest, OnlyProportionTermNonZero)
{
    PidController<double> pid{1.0, 0.0, 0.0, 0.0};

    EXPECT_DOUBLE_EQ(pid.step(0.0, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(pid.step(0.0, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(pid.step(1.0, 1.0), 1.0);
    EXPECT_DOUBLE_EQ(pid.step(8.0, 1.0), 8.0);
    EXPECT_DOUBLE_EQ(pid.step(-5.0, 1.0), -5.0);
    EXPECT_DOUBLE_EQ(pid.step(-1.0, 1.0), -1.0);

    pid = PidController<double>{5.0, 0.0, 0.0, 0.0};

    EXPECT_DOUBLE_EQ(pid.step(0.0, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(pid.step(0.0, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(pid.step(1.0, 1.0), 5.0);
    EXPECT_DOUBLE_EQ(pid.step(8.0, 1.0), 40.0);
    EXPECT_DOUBLE_EQ(pid.step(-5.0, 1.0), -25.0);
    EXPECT_DOUBLE_EQ(pid.step(-1.0, 1.0), -5.0);
}

TEST(PidControllerTest, OnlyIntegralTermNonZero)
{
    constexpr double k_i = 2.0;
    PidController<double> pid{0.0, k_i, 0.0, 10.0};

    EXPECT_DOUBLE_EQ(pid.step(1.0, 1.0), k_i * 1.0);
    EXPECT_DOUBLE_EQ(pid.step(1.0, 1.0), k_i * 2.0);
    EXPECT_DOUBLE_EQ(pid.step(1.0, 1.0), k_i * 3.0);
    EXPECT_DOUBLE_EQ(pid.step(0.5, 1.0), k_i * 3.5);

    // Sign swap should reset integrator to 0, then add -0.2
    EXPECT_DOUBLE_EQ(pid.step(-0.2, 1.0), k_i * -0.2);
    EXPECT_DOUBLE_EQ(pid.step(-1.0, 1.0), k_i * -1.2);
    // Sign swap back to 0.0? No, 0.0 doesn't swap sign usually (0*X is 0, not < 0)
    // My implementation: last_error_.value() * error < T(0.0)
    // If error is 0, it's not < 0.
    EXPECT_DOUBLE_EQ(pid.step(0.0, 1.0), k_i * -1.2);

    // should not accumulate integral term above max_integral
    EXPECT_DOUBLE_EQ(pid.step(6.7, 1.0), k_i * 5.5);  // Sign not swapped from 0.0 to 6.7
    EXPECT_DOUBLE_EQ(pid.step(5.0, 1.0),
                     k_i * 10.0);  // Clamped (5.5 + 5.0 = 10.5 -> 10.0)
    EXPECT_DOUBLE_EQ(pid.step(1.0, 1.0), k_i * 10.0);
}

TEST(PidControllerTest, OnlyDerivativeTermNonZero)
{
    constexpr double k_d = 3.0;
    PidController<double> pid{0.0, 0.0, k_d, 0.0};

    EXPECT_DOUBLE_EQ(pid.step(0.0, 1.0), k_d * 0.0);
    EXPECT_DOUBLE_EQ(pid.step(0.0, 1.0), k_d * 0.0);
    EXPECT_DOUBLE_EQ(pid.step(1.0, 1.0), k_d * 1.0);
    EXPECT_DOUBLE_EQ(pid.step(8.0, 1.0), k_d * 7.0);
    EXPECT_DOUBLE_EQ(pid.step(-5.0, 1.0), k_d * -13.0);
    EXPECT_DOUBLE_EQ(pid.step(0.0, 1.0), k_d * 5.0);
    EXPECT_DOUBLE_EQ(pid.step(5.0, 0.5), k_d * 5.0 / 0.5);
    EXPECT_DOUBLE_EQ(pid.step(8.0, 2.0), k_d * 3.0 / 2.0);
}

TEST(PidControllerTest, GeneralApplication)
{
    constexpr double k_p          = 10.0;
    constexpr double k_i          = 3.0;
    constexpr double k_d          = -1.0;
    constexpr double max_integral = 10.0;
    PidController<double> pid{k_p, k_i, k_d, max_integral};

    EXPECT_DOUBLE_EQ(pid.step(12.0, 0.75), k_p * 12.0 + k_i * 9.0 + k_d * 0.0);
    EXPECT_DOUBLE_EQ(pid.step(24.0, 0.75), k_p * 24.0 + k_i * 10.0 + k_d * 12.0 / 0.75);
    EXPECT_DOUBLE_EQ(pid.step(4.0, 1.0), k_p * 4.0 + k_i * 10.0 + k_d * -20.0);
    EXPECT_DOUBLE_EQ(pid.step(0.0, 1.0), k_p * 0.0 + k_i * 10.0 + k_d * -4.0);
    EXPECT_DOUBLE_EQ(pid.step(2.0, 1.0), k_p * 2.0 + k_i * 10.0 + k_d * 2.0);
    // Sign swap reset: integral becomes 0 + -2.0*1.0 = -2.0
    EXPECT_DOUBLE_EQ(pid.step(-2.0, 1.0), k_p * -2.0 + k_i * -2.0 + k_d * -4.0);
}

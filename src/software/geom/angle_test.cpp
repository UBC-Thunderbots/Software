#include "software/geom/angle.h"

#include <gtest/gtest.h>

#include <limits>

TEST(AngleTest, Statics)
{
    EXPECT_DOUBLE_EQ(0, Angle::zero().toDegrees());
    EXPECT_DOUBLE_EQ(90, Angle::quarter().toDegrees());
    EXPECT_DOUBLE_EQ(180, Angle::half().toDegrees());
    EXPECT_DOUBLE_EQ(270, Angle::threeQuarter().toDegrees());
    EXPECT_DOUBLE_EQ(360, Angle::full().toDegrees());

    EXPECT_DOUBLE_EQ(0, Angle::zero().toRadians());
    EXPECT_DOUBLE_EQ(M_PI_2, Angle::quarter().toRadians());
    EXPECT_DOUBLE_EQ(M_PI, Angle::half().toRadians());
    EXPECT_DOUBLE_EQ(3 * M_PI_2, Angle::threeQuarter().toRadians());
    EXPECT_DOUBLE_EQ(2 * M_PI, Angle::full().toRadians());
}

TEST(AngleTest, From_radians)
{
    EXPECT_DOUBLE_EQ(0, Angle::fromRadians(0).toDegrees());
    EXPECT_DOUBLE_EQ(30, Angle::fromRadians(M_PI / 6).toDegrees());
    EXPECT_DOUBLE_EQ(390, Angle::fromRadians(M_PI * 13 / 6).toDegrees());
}

TEST(AngleTest, From_degrees)
{
    EXPECT_DOUBLE_EQ(0, Angle::fromDegrees(0).toRadians());
    EXPECT_DOUBLE_EQ(M_PI / 3, Angle::fromDegrees(60).toRadians());
    EXPECT_DOUBLE_EQ(M_PI, Angle::fromDegrees(180).toRadians());
    EXPECT_DOUBLE_EQ(-M_PI, Angle::fromDegrees(-180).toRadians());
}

TEST(AngleTest, Angle_clamp)
{
    EXPECT_DOUBLE_EQ(27, Angle::fromDegrees(27).clamp().toDegrees());
    EXPECT_DOUBLE_EQ(27, Angle::fromDegrees(360 + 27).clamp().toDegrees());
    EXPECT_DOUBLE_EQ(-27, Angle::fromDegrees(360 - 27).clamp().toDegrees());
}

TEST(AngleTest, Angle_mod)
{
    EXPECT_DOUBLE_EQ(30, Angle::quarter().mod(Angle::fromDegrees(60)).toDegrees());
    EXPECT_DOUBLE_EQ(0, Angle::zero().mod(Angle::half()).toDegrees());
    EXPECT_DOUBLE_EQ(270, Angle::threeQuarter().mod(Angle::full()).toDegrees());
    EXPECT_DOUBLE_EQ(180, Angle::half().mod(Angle::zero()).toDegrees());
}

TEST(AngleTest, Angle_remainder)
{
    EXPECT_DOUBLE_EQ(0, Angle::full().remainder(Angle::half()).toRadians());
}

TEST(AngleTest, Angle_abs)
{
    EXPECT_DOUBLE_EQ(90, Angle::quarter().abs().toDegrees());
    EXPECT_DOUBLE_EQ(90, (Angle::quarter() - Angle::half()).abs().toDegrees());
}

TEST(AngleTest, Angle_isFinite)
{
    EXPECT_TRUE(Angle::zero().isFinite());
    EXPECT_TRUE(Angle::half().isFinite());

    EXPECT_FALSE(Angle::fromDegrees(NAN).isFinite());
}

TEST(AngleTest, Angle_diff)
{
    EXPECT_DOUBLE_EQ(27,
                     Angle::fromDegrees(50).minDiff(Angle::fromDegrees(23)).toDegrees());
    // We require a slightly larger tolerance for this test to pass
    EXPECT_NEAR(27,
                Angle::fromDegrees(360 + 13).minDiff(Angle::fromDegrees(-14)).toDegrees(),
                1e-13);
    EXPECT_DOUBLE_EQ(
        27,
        Angle::fromDegrees(180 - 13).minDiff(Angle::fromDegrees(-180 + 14)).toDegrees());
    EXPECT_DOUBLE_EQ(
        27,
        Angle::fromDegrees(180 + 13).minDiff(Angle::fromDegrees(-180 - 14)).toDegrees());
    EXPECT_DOUBLE_EQ(
        27,
        Angle::fromDegrees(-180 + 13).minDiff(Angle::fromDegrees(180 - 14)).toDegrees());
    EXPECT_DOUBLE_EQ(
        27,
        Angle::fromDegrees(-180 - 13).minDiff(Angle::fromDegrees(180 + 14)).toDegrees());
}

TEST(AngleTest, sin)
{
    EXPECT_DOUBLE_EQ(0, Angle::zero().sin());
    EXPECT_DOUBLE_EQ(0.5, Angle::fromDegrees(30).sin());
    EXPECT_DOUBLE_EQ(sqrt(2) / 2, Angle::fromDegrees(45).sin());
    EXPECT_DOUBLE_EQ(sqrt(3) / 2, Angle::fromDegrees(60).sin());
    EXPECT_DOUBLE_EQ(1, Angle::quarter().sin());
    EXPECT_NEAR(0, Angle::half().sin(), 1e-7);
    EXPECT_DOUBLE_EQ(-1, Angle::threeQuarter().sin());
    EXPECT_NEAR(0, Angle::full().sin(), 1e-7);
}

TEST(AngleTest, asin)
{
    EXPECT_EQ(Angle::zero(), Angle::asin(0));
    EXPECT_EQ(Angle::fromDegrees(30), Angle::asin(0.5));
    EXPECT_EQ(Angle::fromDegrees(45), Angle::asin(sqrt(2) / 2));
    EXPECT_EQ(Angle::fromDegrees(60), Angle::asin(sqrt(3) / 2));
    EXPECT_EQ(Angle::quarter(), Angle::asin(1));
    EXPECT_EQ(Angle::threeQuarter(), Angle::asin(-1));
}

TEST(AngleTest, cos)
{
    EXPECT_DOUBLE_EQ(1, Angle::zero().cos());
    EXPECT_DOUBLE_EQ(sqrt(3) / 2, Angle::fromDegrees(30).cos());
    EXPECT_DOUBLE_EQ(sqrt(2) / 2, Angle::fromDegrees(45).cos());
    EXPECT_DOUBLE_EQ(0.5, Angle::fromDegrees(60).cos());
    EXPECT_NEAR(0, Angle::quarter().cos(), 1e-7);
    EXPECT_NEAR(-1, Angle::half().cos(), 1e-7);
    EXPECT_NEAR(0, Angle::threeQuarter().cos(), 1e-7);
    EXPECT_NEAR(1, Angle::full().cos(), 1e-7);
}

TEST(AngleTest, acos)
{
    EXPECT_EQ(Angle::zero(), Angle::acos(1));
    EXPECT_EQ(Angle::fromDegrees(60), Angle::acos(0.5));
    EXPECT_EQ(Angle::fromDegrees(45), Angle::acos(sqrt(2) / 2));
    EXPECT_EQ(Angle::fromDegrees(30), Angle::acos(sqrt(3) / 2));
    EXPECT_EQ(Angle::quarter(), Angle::acos(0));
    EXPECT_EQ(Angle::half(), Angle::acos(-1));
}

TEST(AngleTest, tan)
{
    EXPECT_DOUBLE_EQ(0, Angle::zero().tan());
    EXPECT_DOUBLE_EQ(1 / sqrt(3), Angle::fromDegrees(30).tan());
    EXPECT_DOUBLE_EQ(1, Angle::fromDegrees(45).tan());
    EXPECT_DOUBLE_EQ(sqrt(3), Angle::fromDegrees(60).tan());
    EXPECT_NEAR(0, Angle::half().tan(), 1e-7);
    EXPECT_NEAR(0, Angle::full().tan(), 1e-7);
}

TEST(AngleTest, atan)
{
    EXPECT_EQ(Angle::zero(), Angle::atan(0));
    EXPECT_EQ(Angle::fromDegrees(30), Angle::atan(1 / sqrt(3)));
    EXPECT_EQ(Angle::fromDegrees(45), Angle::atan(1));
    EXPECT_EQ(Angle::fromDegrees(60), Angle::atan(sqrt(3)));
}

TEST(AngleTest, operators)
{
    EXPECT_EQ(Angle::threeQuarter(), Angle::quarter() + Angle::half());
    EXPECT_EQ(Angle::half(), Angle::zero() - Angle::half());
    EXPECT_EQ(Angle::full(), Angle::quarter() * 4);
    EXPECT_EQ(Angle::half(), 2 * Angle::quarter());
    EXPECT_EQ(Angle::quarter(), Angle::full() / 4);
    EXPECT_DOUBLE_EQ(2, Angle::full() / Angle::half());
}

TEST(AngleTest, sum_set_angles)
{
    Angle a = Angle::half();
    a += Angle::quarter();
    EXPECT_EQ(Angle::threeQuarter(), a);
}

TEST(AngleTest, subtract_set_angles)
{
    Angle a = Angle::half();
    a -= Angle::quarter();
    EXPECT_EQ(Angle::quarter(), a);
}

TEST(AngleTest, scale_set_angles)
{
    Angle a = Angle::half();
    a *= 2;
    EXPECT_EQ(Angle::full(), a);
}

TEST(AngleTest, divide_set_angles)
{
    Angle a = Angle::half();
    a /= 2;
    EXPECT_EQ(Angle::quarter(), a);
}

TEST(AngleTest, angle_greater_than)
{
    EXPECT_TRUE(Angle::threeQuarter() > Angle::quarter());
    EXPECT_FALSE(Angle::zero() > Angle::full());
}

TEST(AngleTest, angle_less_than)
{
    EXPECT_TRUE(Angle::half() < Angle::full());
    EXPECT_FALSE(Angle::threeQuarter() < Angle::quarter());
}

TEST(AngleTest, angle_geq)
{
    EXPECT_TRUE(Angle::threeQuarter() >= Angle::quarter());
    EXPECT_TRUE(Angle::threeQuarter() >= Angle::threeQuarter());
    EXPECT_FALSE(Angle::half() >= Angle::threeQuarter());
}

TEST(AngleTest, angle_leq)
{
    EXPECT_TRUE(Angle::quarter() <= Angle::quarter());
    EXPECT_TRUE(Angle::quarter() <= Angle::half());
    EXPECT_FALSE(Angle::half() <= Angle::zero());
}

TEST(AngleTest, angle_eq)
{
    EXPECT_TRUE(Angle::threeQuarter() == Angle::threeQuarter());
    EXPECT_TRUE(Angle::fromDegrees(90) == Angle::fromRadians(M_PI_2));
    EXPECT_FALSE(Angle::half() == Angle::threeQuarter());
}

TEST(AngleTest, angle_neq)
{
    EXPECT_TRUE(Angle::threeQuarter() != Angle::quarter());
    EXPECT_FALSE(Angle::half() != Angle::half());
}

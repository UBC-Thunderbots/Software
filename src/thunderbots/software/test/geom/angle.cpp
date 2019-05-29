#include "geom/angle.h"

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

TEST(AngleTest, Of_radians)
{
    EXPECT_DOUBLE_EQ(0, Angle::ofRadians(0).toDegrees());
    EXPECT_DOUBLE_EQ(30, Angle::ofRadians(M_PI / 6).toDegrees());
    EXPECT_DOUBLE_EQ(390, Angle::ofRadians(M_PI * 13 / 6).toDegrees());
}

TEST(AngleTest, Of_degrees)
{
    EXPECT_DOUBLE_EQ(0, Angle::ofDegrees(0).toRadians());
    EXPECT_DOUBLE_EQ(M_PI / 3, Angle::ofDegrees(60).toRadians());
    EXPECT_DOUBLE_EQ(M_PI, Angle::ofDegrees(180).toRadians());
    EXPECT_DOUBLE_EQ(-M_PI, Angle::ofDegrees(-180).toRadians());
}

TEST(AngleTest, Angle_mod)
{
    EXPECT_DOUBLE_EQ(27, Angle::ofDegrees(27).clamp().toDegrees());
    EXPECT_DOUBLE_EQ(27, Angle::ofDegrees(360 + 27).clamp().toDegrees());
    EXPECT_DOUBLE_EQ(-27, Angle::ofDegrees(360 - 27).clamp().toDegrees());
}

TEST(AngleTest, Angle_remainder)
{
    EXPECT_DOUBLE_EQ(0, Angle::full().remainder(Angle::half()).toRadians());
}

TEST(AngleTest, Angle_abs)
{
    EXPECT_DOUBLE_EQ(90, (Angle::quarter() - Angle::half()).abs().toDegrees());
}

TEST(AngleTest, Angle_isFinite)
{
    // TODO: Add tests
}

TEST(AngleTest, Angle_diff)
{
    EXPECT_DOUBLE_EQ(27, Angle::ofDegrees(50).minDiff(Angle::ofDegrees(23)).toDegrees());
    // We require a slightly larger tolerance for this test to pass
    EXPECT_NEAR(27, Angle::ofDegrees(360 + 13).minDiff(Angle::ofDegrees(-14)).toDegrees(),
                1e-13);
    EXPECT_DOUBLE_EQ(
        27, Angle::ofDegrees(180 - 13).minDiff(Angle::ofDegrees(-180 + 14)).toDegrees());
    EXPECT_DOUBLE_EQ(
        27, Angle::ofDegrees(180 + 13).minDiff(Angle::ofDegrees(-180 - 14)).toDegrees());
    EXPECT_DOUBLE_EQ(
        27, Angle::ofDegrees(-180 + 13).minDiff(Angle::ofDegrees(180 - 14)).toDegrees());
    EXPECT_DOUBLE_EQ(
        27, Angle::ofDegrees(-180 - 13).minDiff(Angle::ofDegrees(180 + 14)).toDegrees());
}

TEST(AngleTest, asin)
{
    EXPECT_DOUBLE_EQ(0, Angle::asin(0).toRadians());
    EXPECT_DOUBLE_EQ(M_PI / 2, Angle::asin(1).toRadians());
    EXPECT_DOUBLE_EQ(-M_PI / 2, Angle::asin(-1).toRadians());
    EXPECT_DOUBLE_EQ(30, Angle::asin(0.5).toDegrees());
    EXPECT_NEAR(-2.865983983, Angle::asin(-0.05).toDegrees(), 1e-7);
    EXPECT_NEAR(57.14011962, Angle::asin(0.84).toDegrees(), 1e-7);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

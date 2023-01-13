#include "software/physics/velocity_conversion_util.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(VelocityConversionUtilTest, test_conversion_zero)
{
    Vector global_vector(1, 0);
    Angle global_orientation = Angle::zero();

    Vector expected_local_vector(1, 0);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_local_vector, globalToLocalVelocity(global_vector, global_orientation),
        0.001));

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        global_vector, localToGlobalVelocity(expected_local_vector, global_orientation),
        0.001));
}

TEST(VelocityConversionUtilTest, test_conversion_ninety)
{
    Vector global_vector(1, 1);
    Angle global_orientation = Angle::fromDegrees(90);

    Vector expected_local_vector(1, -1);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_local_vector, globalToLocalVelocity(global_vector, global_orientation),
        0.001));

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        global_vector, localToGlobalVelocity(expected_local_vector, global_orientation),
        0.001));
}

TEST(VelocityConversionUtilTest, test_conversion_greater_than_one_eighty)
{
    Vector global_vector(1, 0);
    Angle global_orientation = Angle::fromDegrees(225);

    Vector expected_local_vector(-1 / std::sqrt(2), 1 / std::sqrt(2));

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_local_vector, globalToLocalVelocity(global_vector, global_orientation),
        0.001));

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        global_vector, localToGlobalVelocity(expected_local_vector, global_orientation),
        0.001));
}

TEST(VelocityConversionUtilTest, test_conversion_negative)
{
    Vector global_vector(1, 0);
    Angle global_orientation = Angle::fromDegrees(-45);

    Vector expected_local_vector(1 / std::sqrt(2), 1 / std::sqrt(2));

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_local_vector, globalToLocalVelocity(global_vector, global_orientation),
        0.001));

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        global_vector, localToGlobalVelocity(expected_local_vector, global_orientation),
        0.001));
}

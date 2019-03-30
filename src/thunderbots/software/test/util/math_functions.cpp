//
// Created by roark on 28/03/19.
//

#include "util/math_functions.h"

#include <gtest/gtest.h>

TEST(LinearUtilFunctionTest, testZeroCase)
{
    double out = Util::linear(0, 0, 2);
    EXPECT_EQ(out, 0.5);
}

TEST(LinearUtilFunctionTest, testOneQuarter)
{
    double out = Util::linear(-1, 0, 4);
    EXPECT_EQ(out, 0.25);
}

TEST(LinearUtilFunctionTest, testTwoThirds)
{
    double out = Util::linear(0.75, 0, 4.5);
    EXPECT_EQ(out, 2.0 / 3.0);
}

TEST(LinearUtilFunctionTest, testMinimumNoOffset)
{
    double out = Util::linear(-1.5, 0, 3);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testMaximumNoOffset)
{
    double out = Util::linear(2.5, 0, 5.0);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testClampBelowNoOffset)
{
    double out = Util::linear(-2, 0, 1);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testClampAboveNoOffset)
{
    double out = Util::linear(4.2, 0, 6);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testMinimumNegativeOffset)
{
    double out = Util::linear(-4, -2, 4);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testMaximumNegativeOffset)
{
    double out = Util::linear(1.5, -1, 5.0);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testMinimumPositiveOffset)
{
    double out = Util::linear(0, 3, 6);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testMaximumPositiveOffset)
{
    double out = Util::linear(6, 1.5, 9);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testClampBelowNegativeOffset)
{
    double out = Util::linear(-1.8, -0.2, 0.7);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testClampAboveNegativeOffset)
{
    double out = Util::linear(0.18, -0.05, 0.24);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testClampBelowPositiveOffset)
{
    double out = Util::linear(-0.3, 0.6, 1.3);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testClampAbovePositiveOffset)
{
    double out = Util::linear(0.35, 0.15, 0.18);
    EXPECT_EQ(out, 1.0);
}


int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

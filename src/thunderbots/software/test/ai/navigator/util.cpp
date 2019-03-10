#include "ai/navigator/util.h"

#include <gtest/gtest.h>
#include <math.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

#include "geom/point.h"

TEST(NavUtilTest, calculateTransitionSpeedBetweenSegments_tests_parallel_segments)
{
    Point testp1, testp2, testp3;
    double final_speed;
    // case 1
    testp1      = Point(1, 0);
    testp2      = Point(2, 0);
    testp3      = Point(3, 0);
    final_speed = 2.2;
    EXPECT_DOUBLE_EQ(final_speed, calculateTransitionSpeedBetweenSegments(
                                      testp1, testp2, testp3, final_speed));

    // case 2
    testp1      = Point(1, 1);
    testp2      = Point(1, 2);
    testp3      = Point(1, 3);
    final_speed = -2.2;
    EXPECT_DOUBLE_EQ(final_speed, calculateTransitionSpeedBetweenSegments(
                                      testp1, testp2, testp3, final_speed));
}

TEST(NavUtilTest, calculateTransitionSpeedBetweenSegments_tests_perpendicular_segments)
{
    Point testp1, testp2, testp3;
    double final_speed;
    // case 1
    testp1      = Point(0, 1);
    testp2      = Point(1, 1);
    testp3      = Point(1, 2);
    final_speed = -2.2;
    EXPECT_DOUBLE_EQ(
        0, calculateTransitionSpeedBetweenSegments(testp1, testp2, testp3, final_speed));

    // case 2
    testp1      = Point(1, 0);
    testp2      = Point(2, 0);
    testp3      = Point(2, 1);
    final_speed = 2.2;
    EXPECT_DOUBLE_EQ(
        0, calculateTransitionSpeedBetweenSegments(testp1, testp2, testp3, final_speed));
}


TEST(NavUtilTest, calculateTransitionSpeedBetweenSegments_tests_nan_corner_cases)
{
    Point testp1, testp2, testp3;
    double final_speed;
    // case 1
    testp1      = Point(0, 1);
    testp2      = Point(0, 1);
    testp3      = Point(1, 2);
    final_speed = -2.2;
    EXPECT_FALSE(isnormal(
        calculateTransitionSpeedBetweenSegments(testp1, testp2, testp3, final_speed)));

    // case 2
    testp1      = Point(1, 0);
    testp2      = Point(2, 0);
    testp3      = Point(2, 0);
    final_speed = 2.2;
    EXPECT_FALSE(isnormal(
        calculateTransitionSpeedBetweenSegments(testp1, testp2, testp3, final_speed)));
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

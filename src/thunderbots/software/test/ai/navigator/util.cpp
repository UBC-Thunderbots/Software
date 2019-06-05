#include "ai/navigator/util.h"

#include <gtest/gtest.h>
#include <math.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

#include "geom/point.h"
#include "geom/util.h"

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

TEST(getPointTrespassTest, distance_within_threshold_test)
{
    const Point &p1  = Point(0, 0);
    const Point &p2  = Point(0, 4);
    double threshold = 6.0;

    EXPECT_EQ(2, getPointTrespass(p1, p2, threshold));
}

TEST(getPointTrespassTest, distance_closer_within_threshold_test)
{
    const Point &p1  = Point(0, 2);
    const Point &p2  = Point(0, 3);
    double threshold = 6.0;

    EXPECT_EQ(5, getPointTrespass(p1, p2, threshold));
}

TEST(getPointTrespassTest, distance_equals_threshold_test)
{
    const Point &p1  = Point(0, 0);
    const Point &p2  = Point(0, 3);
    double threshold = 3.0;

    EXPECT_EQ(0, getPointTrespass(p1, p2, threshold));
}

TEST(getPointTrespassTest, distance_greater_than_threshold_test)
{
    const Point &p1  = Point(0, 0);
    const Point &p2  = Point(0, 3);
    double threshold = 1.0;

    EXPECT_EQ(0, getPointTrespass(p1, p2, threshold));
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

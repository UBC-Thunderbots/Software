#include "ai/navigator/util.h"

#include <gtest/gtest.h>
#include <math.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

#include "geom/point.h"

TEST(NavUtilTest, calculateTransitionVelocityBetweenSegments_tests_parallel_segments)
{
    Point testp1, testp2, testp3;
    double final_vel;
    // case 1
    testp1    = Point(1, 0);
    testp2    = Point(2, 0);
    testp3    = Point(3, 0);
    final_vel = 2.2;
    EXPECT_DOUBLE_EQ(final_vel, calculateTransitionVelocityBetweenSegments(
                                    testp1, testp2, testp3, final_vel));

    // case 2
    testp1    = Point(1, 1);
    testp2    = Point(1, 2);
    testp3    = Point(1, 3);
    final_vel = -2.2;
    EXPECT_DOUBLE_EQ(final_vel, calculateTransitionVelocityBetweenSegments(
                                    testp1, testp2, testp3, final_vel));
}

TEST(NavUtilTest, calculateTransitionVelocityBetweenSegments_tests_perpendicular_segments)
{
    Point testp1, testp2, testp3;
    double final_vel;
    // case 1
    testp1    = Point(0, 1);
    testp2    = Point(1, 1);
    testp3    = Point(1, 2);
    final_vel = -2.2;
    EXPECT_DOUBLE_EQ(
        0, calculateTransitionVelocityBetweenSegments(testp1, testp2, testp3, final_vel));

    // case 2
    testp1    = Point(1, 0);
    testp2    = Point(2, 0);
    testp3    = Point(2, 1);
    final_vel = 2.2;
    EXPECT_DOUBLE_EQ(
        0, calculateTransitionVelocityBetweenSegments(testp1, testp2, testp3, final_vel));
}


TEST(NavUtilTest, calculateTransitionVelocityBetweenSegments_tests_nan_corner_cases)
{
    Point testp1, testp2, testp3;
    double final_vel;
    // case 1
    testp1    = Point(0, 1);
    testp2    = Point(0, 1);
    testp3    = Point(1, 2);
    final_vel = -2.2;
    EXPECT_FALSE(isnormal(
        calculateTransitionVelocityBetweenSegments(testp1, testp2, testp3, final_vel)));

    // case 2
    testp1    = Point(1, 0);
    testp2    = Point(2, 0);
    testp3    = Point(2, 0);
    final_vel = 2.2;
    EXPECT_FALSE(isnormal(
        calculateTransitionVelocityBetweenSegments(testp1, testp2, testp3, final_vel)));
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

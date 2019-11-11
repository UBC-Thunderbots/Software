#include "software/ai/navigator/util.h"

#include <gtest/gtest.h>
#include <math.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <sstream>

#include "software/geom/util.h"
#include "software/new_geom/point.h"

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

TEST(NavUtilTest, convertPointsToMovePrimitives_test)
{
    unsigned int robot_id = 1;

    Point point1 = Point(0, 1);
    Point point2 = Point(1, 3);
    Point point3 = Point(2, 5);

    std::vector<Point> points = {point1, point2, point3};

    std::vector<MovePrimitive> movePrimitives = convertToMovePrimitives(
        robot_id, points, DribblerEnable::OFF, AutokickType::NONE);

    // testing point 1
    MovePrimitive movePrimitive1 = movePrimitives.at(0);
    EXPECT_EQ(movePrimitive1.getRobotId(), robot_id);
    EXPECT_EQ(movePrimitive1.getDestination(), point1);
    EXPECT_EQ(movePrimitive1.getFinalSpeed(), 0);
    EXPECT_EQ(movePrimitive1.getFinalAngle(), point1.toVector().orientation());

    // testing point 2
    MovePrimitive movePrimitive2 = movePrimitives.at(1);
    EXPECT_EQ(movePrimitive2.getRobotId(), robot_id);
    EXPECT_EQ(movePrimitive2.getDestination(), point2);
    EXPECT_EQ(movePrimitive2.getFinalSpeed(), 0);
    EXPECT_EQ(movePrimitive2.getFinalAngle(), point2.toVector().orientation());

    // testing point 3
    MovePrimitive movePrimitive3 = movePrimitives.at(2);
    EXPECT_EQ(movePrimitive3.getRobotId(), 1);
    EXPECT_EQ(movePrimitive3.getDestination(), point3);
    EXPECT_EQ(movePrimitive3.getFinalSpeed(), 0);
    EXPECT_EQ(movePrimitive3.getFinalAngle(), point3.toVector().orientation());
}

TEST(NavUtilTest, convertNoPointsToMovePrimitives_test)
{
    unsigned int robot_id = 1;

    std::vector<Point> points                 = {};
    std::vector<MovePrimitive> movePrimitives = convertToMovePrimitives(
        robot_id, points, DribblerEnable::OFF, AutokickType::NONE);

    EXPECT_THROW(movePrimitives.at(0), std::out_of_range);
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

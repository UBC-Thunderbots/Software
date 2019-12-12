#include "software/new_geom/util/distance.h"

#include <gtest/gtest.h>

#include "software/new_geom/line.h"
#include "software/new_geom/point.h"

TEST(DistanceTest, dist_line_point1)
{
    // TODO: break this up
    double calculated_val, expected_val;

    // case 1
    Line test1line(Point(0, 1), Point(0, 0));
    Point test1p(2, 0);
    calculated_val = distance(test1line, test1p);
    expected_val   = 2;
    EXPECT_EQ(expected_val, calculated_val);

    // case 2
    Line test2line(Point(2, 0), Point(0, 2));
    Point test2p(0, 0);
    calculated_val = distance(test2line, test2p);
    expected_val   = sqrt(2);
    EXPECT_DOUBLE_EQ(expected_val, calculated_val);

    // TODO: test degenerate case
    //    Line test3line(Point(0, 0), Point(0, 0));
    //    Point test3p(1, 0);
    //    calculated_val = distance(test3line, test3p);
    //    expected_val   = 1;
    //    EXPECT_EQ(expected_val, calculated_val);

    Line line(Point(1, -1), Point(5, -2));
    Point p(2, -3);

    EXPECT_NEAR(1.69775, distance(line, p), 1e-5);

    p = Point(2, 1);
    EXPECT_NEAR(2.18282, distance(line, p), 1e-5);

    p = Point(2, 0);
    EXPECT_NEAR(1.21268, distance(line, p), 1e-5);
}

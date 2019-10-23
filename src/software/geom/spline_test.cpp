#include "software/geom/spline.h"

#include <gtest/gtest.h>

TEST(TestSpline, test_spline_points_constructor)
{
    std::vector<Point> points({Point(4, 18), Point(3, 0), Point(-7, -1)});
    Spline s(points);
    EXPECT_EQ(s.size(), points.size());
    std::vector<Point> spline_points = s.getKnots();
    EXPECT_EQ(spline_points, points);
    EXPECT_EQ(s.valueAt(0.0), points[0]);
    EXPECT_EQ(s.startPoint(), points[0]);
    EXPECT_EQ(s.valueAt(0.1), Point(3.8, 14.4));
    EXPECT_EQ(s.valueAt(0.5), points[1]);
    EXPECT_EQ(s.valueAt(0.6), Point(1, -0.2));
    EXPECT_EQ(s.valueAt(1.0), points[2]);
    EXPECT_EQ(s.endPoint(), points[2]);

    bool failing = true;

    try
    {
        s.valueAt(-0.1);
    }
    catch (std::invalid_argument &e)
    {
        failing = false;
    }

    if (failing)
    {
        ADD_FAILURE() << "Spline returning value for value (-0.1) outside of domain";
        return;
    }

    try
    {
        s.valueAt(1.1);
    }
    catch (std::invalid_argument &e)
    {
        failing = false;
    }

    if (failing)
    {
        ADD_FAILURE() << "Spline returning value for value (1.1) outside of domain";
        return;
    }
}

TEST(TestSpline, test_polynomial_points_list_constructor)
{
    std::vector<Point> points({Point(1, 2), Point(2, 3), Point(0, -1)});
    Spline s({Point(1, 2), Point(2, 3), Point(0, -1)});
    EXPECT_EQ(s.size(), points.size());
    std::vector<Point> spline_points = s.getKnots();
    EXPECT_EQ(spline_points, points);
    EXPECT_EQ(s.valueAt(0.0), points[0]);
    EXPECT_EQ(s.startPoint(), points[0]);
    EXPECT_EQ(s.valueAt(0.1), Point(1.2, 2.2));
    EXPECT_EQ(s.valueAt(0.5), points[1]);
    EXPECT_EQ(s.valueAt(0.6), Point(1.6, 2.2));
    EXPECT_EQ(s.valueAt(1.0), points[2]);
    EXPECT_EQ(s.endPoint(), points[2]);

    bool failing = true;

    try
    {
        s.valueAt(-0.1);
    }
    catch (std::invalid_argument &e)
    {
        failing = false;
    }

    if (failing)
    {
        ADD_FAILURE() << "Spline returning value for value (-0.1) outside of domain";
        return;
    }

    try
    {
        s.valueAt(1.1);
    }
    catch (std::invalid_argument &e)
    {
        failing = false;
    }

    if (failing)
    {
        ADD_FAILURE() << "Spline returning value for value (1.1) outside of domain";
        return;
    }
}

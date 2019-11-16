#include "software/geom/spline.h"

#include <gtest/gtest.h>

TEST(TestSpline, test_three_knot_value_at)
{
    std::vector<Point> points({Point(4, 18), Point(3, 0), Point(-7, -1)});
    Spline s(points);
    EXPECT_EQ(s.size(), points.size());
    EXPECT_EQ(s.valueAt(0.0), points[0]);
    EXPECT_TRUE(s.valueAt(0.1).isClose(Point(3.8, 14.4), 1e-9));
    EXPECT_TRUE(s.valueAt(0.5).isClose(points[1], 1e-9));
    EXPECT_TRUE(s.valueAt(0.6).isClose(Point(1, -0.2), 1e-8));
    EXPECT_TRUE(s.valueAt(0.9).isClose(Point(-5, -0.8), 1e-9));
    EXPECT_EQ(s.valueAt(1.0), points[2]);
}

TEST(TestSpline, test_one_knot_value_at)
{
    std::vector<Point> points({Point(3, 7)});
    Spline s(points);
    EXPECT_EQ(s.size(), points.size());
    EXPECT_EQ(s.valueAt(0.0), points[0]);
    EXPECT_EQ(s.valueAt(0.1), points[0]);
    EXPECT_EQ(s.valueAt(0.5), points[0]);
    EXPECT_EQ(s.valueAt(0.6), points[0]);
    EXPECT_EQ(s.valueAt(0.9), points[0]);
    EXPECT_EQ(s.valueAt(1.0), points[0]);
}

TEST(TestSpline, test_double_repeated_knot_value_at)
{
    std::vector<Point> points({Point(-3, 20), Point(-3, 20)});
    Spline s(points);
    EXPECT_EQ(s.size(), points.size());
    EXPECT_EQ(s.valueAt(0.0), points[0]);
    EXPECT_EQ(s.valueAt(0.1), points[0]);
    EXPECT_EQ(s.valueAt(0.5), points[0]);
    EXPECT_EQ(s.valueAt(0.6), points[0]);
    EXPECT_EQ(s.valueAt(0.9), points[0]);
    EXPECT_EQ(s.valueAt(1.0), points[0]);
}

TEST(TestSpline, test_two_knot_value_at)
{
    std::vector<Point> points({Point(-3, -21), Point(4, 32)});
    Spline s(points);
    EXPECT_EQ(s.size(), points.size());
    EXPECT_EQ(s.valueAt(0.0), points[0]);
    EXPECT_TRUE(s.valueAt(0.1).isClose(Point(-2.3, -15.7), 1e-9));
    EXPECT_TRUE(s.valueAt(0.5).isClose(Point(0.5, 5.5), 1e-9));
    EXPECT_TRUE(s.valueAt(0.6).isClose(Point(1.2, 10.8), 1e-7));
    EXPECT_TRUE(s.valueAt(0.9).isClose(Point(3.3, 26.7), 1e-7));
    EXPECT_EQ(s.valueAt(1.0), points[1]);
}

TEST(TestSpline, test_three_knot_value_at_out_of_range)
{
    std::vector<Point> points({Point(4, 18), Point(3, 0), Point(-7, -1)});
    Spline s(points);

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

TEST(TestSpline, test_one_knot_value_at_out_of_range)
{
    std::vector<Point> points({Point(1, -18)});
    Spline s(points);

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

TEST(TestSpline, test_two_knot_value_at_out_of_range)
{
    std::vector<Point> points({Point(1, -2), Point(1, -18)});
    Spline s(points);

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

TEST(TestSpline, test_spline_points_list_constructor)
{
    std::vector<Point> points({Point(1, 2), Point(2, 3), Point(0, -1)});
    Spline s({Point(1, 2), Point(2, 3), Point(0, -1)});
    EXPECT_EQ(s.size(), points.size());
    std::vector<Point> spline_points = s.getKnots();
    EXPECT_EQ(spline_points, points);
    EXPECT_EQ(s.startPoint(), points[0]);
    EXPECT_EQ(s.endPoint(), points[2]);
}

TEST(TestSpline, test_spline_points_vector_constructor)
{
    std::vector<Point> points({Point(1, 2), Point(2, 3), Point(0, -1)});
    Spline s(points);
    EXPECT_EQ(s.size(), points.size());
    std::vector<Point> spline_points = s.getKnots();
    EXPECT_EQ(spline_points, points);
    EXPECT_EQ(s.startPoint(), points[0]);
    EXPECT_EQ(s.endPoint(), points[2]);
}

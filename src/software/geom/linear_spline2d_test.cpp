#include "software/geom/linear_spline2d.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(LinearSpline2dTest, test_constructor_three_knot)
{
    std::vector<Point> points({Point(4, 18), Point(3, 0), Point(-7, -1)});
    LinearSpline2d s(points);
    EXPECT_EQ(s.getNumKnots(), points.size());
    EXPECT_EQ(s.getValueAt(0.0), points[0]);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(s.getValueAt(0.1), Point(3.8, 14.4), 1e-9));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(s.getValueAt(0.5), points[1], 1e-9));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(s.getValueAt(0.6), Point(1, -0.2), 1e-8));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(s.getValueAt(0.9), Point(-5, -0.8), 1e-9));
    EXPECT_EQ(s.getValueAt(1.0), points[2]);
}

TEST(LinearSpline2dTest, test_constructor_one_knot)
{
    std::vector<Point> points({Point(3, 7)});
    LinearSpline2d s(points);
    EXPECT_EQ(s.getNumKnots(), points.size());
    EXPECT_EQ(s.getValueAt(0.0), points[0]);
    EXPECT_EQ(s.getValueAt(0.1), points[0]);
    EXPECT_EQ(s.getValueAt(0.5), points[0]);
    EXPECT_EQ(s.getValueAt(0.6), points[0]);
    EXPECT_EQ(s.getValueAt(0.9), points[0]);
    EXPECT_EQ(s.getValueAt(1.0), points[0]);
}

TEST(LinearSpline2dTest, test_constructor_double_repeated_knot)
{
    std::vector<Point> points({Point(-3, 20), Point(-3, 20)});
    LinearSpline2d s(points);
    EXPECT_EQ(s.getNumKnots(), points.size());
    EXPECT_EQ(s.getValueAt(0.0), points[0]);
    EXPECT_EQ(s.getValueAt(0.1), points[0]);
    EXPECT_EQ(s.getValueAt(0.5), points[0]);
    EXPECT_EQ(s.getValueAt(0.6), points[0]);
    EXPECT_EQ(s.getValueAt(0.9), points[0]);
    EXPECT_EQ(s.getValueAt(1.0), points[0]);
}

TEST(LinearSpline2dTest, test_constructor_two_knot)
{
    std::vector<Point> points({Point(-3, -21), Point(4, 32)});
    LinearSpline2d s(points);
    EXPECT_EQ(s.getNumKnots(), points.size());
    EXPECT_EQ(s.getValueAt(0.0), points[0]);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(s.getValueAt(0.1), Point(-2.3, -15.7), 1e-9));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(s.getValueAt(0.5), Point(0.5, 5.5), 1e-9));
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(s.getValueAt(0.6), Point(1.2, 10.8), 1e-7));
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(s.getValueAt(0.9), Point(3.3, 26.7), 1e-7));
    EXPECT_EQ(s.getValueAt(1.0), points[1]);
}

TEST(LinearSpline2dTest, test_one_knot_value_at_out_of_range)
{
    std::vector<Point> points({Point(1, -18)});
    LinearSpline2d s(points);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(s.getValueAt(-0.1), Point(1, -18), 1e-9));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(s.getValueAt(1.1), Point(1, -18), 1e-9));
}


TEST(LinearSpline2dTest, test_two_knot_value_at_out_of_range)
{
    std::vector<Point> points({Point(1, -2), Point(1, -18)});
    LinearSpline2d s(points);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(s.getValueAt(-0.1), Point(1, -2), 1e-9));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(s.getValueAt(1.1), Point(1, -18), 1e-9));
}


TEST(LinearSpline2dTest, test_three_knot_value_at_out_of_range)
{
    std::vector<Point> points({Point(4, 18), Point(3, 0), Point(-7, -1)});
    LinearSpline2d s(points);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(s.getValueAt(-0.1), Point(4, 18), 1e-9));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(s.getValueAt(1.1), Point(-7, -1), 1e-9));
}

TEST(LinearSpline2dTest, test_spline_points_list_constructor)
{
    std::vector<Point> points({Point(1, 2), Point(2, 3), Point(0, -1)});
    LinearSpline2d s({Point(1, 2), Point(2, 3), Point(0, -1)});
    EXPECT_EQ(s.getNumKnots(), points.size());
    std::vector<Point> spline_points = s.getKnots();
    EXPECT_EQ(spline_points, points);
    EXPECT_EQ(s.getStartPoint(), points[0]);
    EXPECT_EQ(s.getEndPoint(), points[2]);
}

TEST(LinearSpline2dTest, test_spline_points_vector_constructor)
{
    std::vector<Point> points({Point(1, 2), Point(2, 3), Point(0, -1)});
    LinearSpline2d s(points);
    EXPECT_EQ(s.getNumKnots(), points.size());
    std::vector<Point> spline_points = s.getKnots();
    EXPECT_EQ(spline_points, points);
    EXPECT_EQ(s.getStartPoint(), points[0]);
    EXPECT_EQ(s.getEndPoint(), points[2]);
}

TEST(LinearSpline2dTest, test_get_knot_parametrization_values)
{
    LinearSpline2d s({Point(1, 2), Point(2, 3), Point(0, -1)});

    EXPECT_EQ(std::vector<double>({0, 0.5, 1}), s.getKnotParametrizationValues());
}

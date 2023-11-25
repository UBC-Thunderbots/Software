#include "software/geom/bezier_curve2d.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(BezierCurve2dTest, contstruct_with_less_then_two_control_points)
{
    std::vector<Point> points = {Point(0, 0)};

    EXPECT_THROW(BezierCurve2d curve(points), std::invalid_argument);
}

TEST(BezierCurve2dTest, get_value_two_points)
{
    // Linear bezier curve
    Point p0(-1, 2);
    Point p1(5, -7);
    const std::vector<Point> points = {p0, p1};

    const BezierCurve2d curve(points);

    const Polynomial2d expected_polynomial2d({p0, p1});

    // We approximate an equality check by sampling at a bunch of points and comparing
    // to the expected polynomial
    for (int i = 0; i <= 1000; i++)
    {
        const double t = static_cast<double>(i) / 1000.0;
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected_polynomial2d.getValueAt(t),
                                                   curve.getValueAt(t), 1e-9));
    }
}

TEST(BezierCurve2dTest, get_value_two_points_x_unchanged)
{
    // Check for potential edge cases where one of x/y is unchanged over the length
    // of the curve

    // Linear bezier curve
    Point p0(0, 2);
    Point p1(0, -7);
    const std::vector<Point> points = {p0, p1};

    const BezierCurve2d curve(points);

    const Polynomial2d expected_polynomial2d({p0, p1});

    // We approximate an equality check by sampling at a bunch of points and comparing
    // to the expected polynomial
    for (int i = 0; i <= 1000; i++)
    {
        const double t = static_cast<double>(i) / 1000.0;
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected_polynomial2d.getValueAt(t),
                                                   curve.getValueAt(t), 1e-9));
    }
}

TEST(BezierCurve2dTest, get_value_below_zero)
{
    // Linear bezier curve
    Point p0(-1, 2);
    Point p1(5, -7);
    const std::vector<Point> points = {p0, p1};

    const BezierCurve2d curve(points);

    // Check that getting a value below zero just returns the first point
    EXPECT_EQ(p0, curve.getValueAt(-0.1));
}

TEST(BezierCurve2dTest, get_value_above_one)
{
    // Linear bezier curve
    Point p0(-1, 2);
    Point p1(5, -7);
    const std::vector<Point> points = {p0, p1};

    const BezierCurve2d curve(points);

    // Check that getting a value just above one just returns the last point
    EXPECT_EQ(p1, curve.getValueAt(1.1));
}

TEST(BezierCurveTest, get_value_four_points)
{
    // Cubic bezier curve
    Point p0(-1, 2);
    Point p1(5, -1);
    Point p2(3, 2);
    Point p3(2, 1);
    const std::vector<Point> points = {p0, p1, p2, p3};

    const BezierCurve2d curve(points);

    // clang-format off
    const Polynomial2d expected_polynomial2d(
        Polynomial1d({
                         p0.x(),
                         3.0 * (p1.x() - p0.x()),
                         3.0 * (p0.x() - 2.0 * p1.x() + p2.x()),
                         -p0.x() + 3.0 * p1.x() - 3.0 * p2.x() + p3.x()
                     }),
        Polynomial1d({
                         p0.y(),
                         3.0 * (p1.y() - p0.y()),
                         3.0 * (p0.y() - 2.0 * p1.y() + p2.y()),
                         -p0.y() + 3.0 * p1.y() - 3.0 * p2.y() + p3.y()
                     }));
    // clang-format on

    // We approximate an equality check by sampling at a bunch of points and comparing
    // to the expected polynomial
    for (int i = 0; i <= 1000; i++)
    {
        const double t = static_cast<double>(i) / 1000.0;
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected_polynomial2d.getValueAt(t),
                                                   curve.getValueAt(t), 1e-9));
    }
}

TEST(BezierCurve2dTest, get_polynomial_two_points)
{
    // Test constructing a bezier curve between two points, ie. a linear bezier curve
    Point p0(-1, 2);
    Point p1(5, -7);
    const std::vector<Point> points = {p0, p1};

    const BezierCurve2d curve(points);

    const Polynomial2d actual_polynomial2d = curve.getPolynomial();
    const Polynomial2d expected_polynomial2d({p0, p1});

    EXPECT_EQ(expected_polynomial2d, actual_polynomial2d);
}

TEST(BezierCurve2dTest, get_polynomial_four_points)
{
    // Test constructing a bezier curve between four points, ie. a cubic bezier curve
    Point p0(-1, 2);
    Point p1(5, -7);
    Point p2(10, 5);
    Point p3(-10, 1);
    const std::vector<Point> points = {Point(p0), Point(p1), Point(p2), Point(p3)};

    const BezierCurve2d curve(points);

    // clang-format off
    const Polynomial2d expected_polynomial2d(
        Polynomial1d({
                         p0.x(),
                         3.0 * (p1.x() - p0.x()),
                         3.0 * (p0.x() - 2.0 * p1.x() + p2.x()),
                         -p0.x() + 3.0 * p1.x() - 3.0 * p2.x() + p3.x()
                     }),
        Polynomial1d({
                         p0.y(),
                         3.0 * (p1.y() - p0.y()),
                         3.0 * (p0.y() - 2.0 * p1.y() + p2.y()),
                         -p0.y() + 3.0 * p1.y() - 3.0 * p2.y() + p3.y()
                     }));
    // clang-format on
    const Polynomial2d actual_polynomial2d = curve.getPolynomial();

    EXPECT_EQ(expected_polynomial2d, actual_polynomial2d);
}

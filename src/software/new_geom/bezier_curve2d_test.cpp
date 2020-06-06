#include "software/new_geom/bezier_curve2d.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

using namespace Test;

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
    // too the expected polynomial
    for (int i = 0; i <= 1000; i++)
    {
        const double t = static_cast<double>(i) / 1000.0;
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected_polynomial2d.valueAt(t),
                                                   curve.getValueAt(t), 1e-9));
    }
}

TEST(BezierCurveTest, get_value_four_points)
{
    // Cubic bezier curve
    Vector v0(-1, 2);
    Vector v1(5, -1);
    Vector v2(3, 2);
    Vector v3(2, 1);
    const std::vector<Point> points = {Point(v0), Point(v1), Point(v2), Point(v3)};

    const BezierCurve2d curve(points);

    // clang-format off
    const Polynomial2d expected_polynomial2d(
        Polynomial1d({
                         v0.x(),
                         3.0 * (v1.x() - v0.x()),
                         3.0 * (v0.x() - 2.0 * v1.x() + v2.x()),
                         -v0.x() + 3.0 * v1.x() - 3.0 * v2.x() + v3.x()
                     }),
        Polynomial1d({
                         v0.y(),
                         3.0 * (v1.y() - v0.y()),
                         3.0 * (v0.y() - 2.0 * v1.y() + v2.y()),
                         -v0.y() + 3.0 * v1.y() - 3.0 * v2.y() + v3.y()
                     }));
    // clang-format on

    // We approximate an equality check by sampling at a bunch of points and comparing
    // too the expected polynomial
    for (int i = 0; i <= 1000; i++)
    {
        const double t = static_cast<double>(i) / 1000.0;
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected_polynomial2d.valueAt(t),
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
    Vector v0(-1, 2);
    Vector v1(5, -7);
    Vector v2(10, 5);
    Vector v3(-10, 1);
    const std::vector<Point> points = {Point(v0), Point(v1), Point(v2), Point(v3)};

    const BezierCurve2d curve(points);

    // clang-format off
    const Polynomial2d expected_polynomial2d(
        Polynomial1d({
                         v0.x(),
                         3.0 * (v1.x() - v0.x()),
                         3.0 * (v0.x() - 2.0 * v1.x() + v2.x()),
                         -v0.x() + 3.0 * v1.x() - 3.0 * v2.x() + v3.x()
                     }),
        Polynomial1d({
                         v0.y(),
                         3.0 * (v1.y() - v0.y()),
                         3.0 * (v0.y() - 2.0 * v1.y() + v2.y()),
                         -v0.y() + 3.0 * v1.y() - 3.0 * v2.y() + v3.y()
                     }));
    // clang-format on
    const Polynomial2d actual_polynomial2d = curve.getPolynomial();

    EXPECT_EQ(expected_polynomial2d, actual_polynomial2d);
}

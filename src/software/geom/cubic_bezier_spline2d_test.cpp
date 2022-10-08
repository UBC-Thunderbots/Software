#include "software/geom/cubic_bezier_spline2d.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/convex_angle.h"
#include "software/geom/bezier_curve2d.h"
#include "software/test_util/test_util.h"

class CubicBezierSplineTest : public ::testing::Test
{
   protected:
    CubicBezierSplineTest()
        : test_spline_1(Point(1, -1), Vector(-3, -4), Point(1, -2), Vector(-2, -5),
                        {
                            Point(5, 5),
                            Point(10, -10),
                        })
    {
    }

    void SetUp() override {}

    void TearDown() override {}

    // In order to avoid duplicate work between tests we create splines here that are
    // used across many tests
    const CubicBezierSpline2d test_spline_1;
};

TEST_F(CubicBezierSplineTest, getNumKnots)
{
    // Two intermediate knots + start point + end point
    EXPECT_EQ(4, test_spline_1.getNumKnots());
}

TEST_F(CubicBezierSplineTest, getKnots)
{
    const std::vector<Point> expected_knots = {Point(1, -1), Point(5, 5), Point(10, -10),
                                               Point(1, -2)};
    EXPECT_EQ(expected_knots, test_spline_1.getKnots());
}

TEST_F(CubicBezierSplineTest, getKnotParametrizationValues)
{
    const std::vector<Point> expected_knots = {Point(1, -1), Point(5, 5), Point(10, -10),
                                               Point(1, -2)};
    const std::vector<double> knot_vector = test_spline_1.getKnotParametrizationValues();

    ASSERT_EQ(4, knot_vector.size());

    for (size_t i = 0; i < knot_vector.size(); i++)
    {
        EXPECT_EQ(test_spline_1.getValueAt(knot_vector[i]), expected_knots[i]);
    }
    EXPECT_EQ(expected_knots, test_spline_1.getKnots());
}

TEST_F(CubicBezierSplineTest, getValueAt__start_point)
{
    // Check that the start point is at the correct position
    const Point start_point = test_spline_1.getValueAt(0);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(start_point, Point(1, -1), 1e-9));

    // Check that the tangent at the start is (approximately) what we expected. This
    // serves to check that we're getting the value for intermediate points correctly
    // as well.
    const Point just_after_start_point   = test_spline_1.getValueAt(1e-9);
    const Vector approx_tangent_at_start = just_after_start_point - start_point;
    const Angle tangent_error_angle = convexAngle(approx_tangent_at_start, Vector(-3, -4));

    EXPECT_NEAR(0, tangent_error_angle.toRadians(), 1e-9);
}

TEST_F(CubicBezierSplineTest, getValueAt__end_point)
{
    // Check that the end point is at the correct position
    const Point end_point = test_spline_1.getValueAt(1);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(end_point, Point(1, -2), 1e-9));

    // Check that the tangent at the end is (approximately) what we expected. This
    // serves to check that we're getting the value for intermediate points correctly
    // as well.
    const Point just_before_end_point  = test_spline_1.getValueAt(1 - 1e-9);
    const Vector approx_tangent_at_end = just_before_end_point - end_point;
    const Angle tangent_error_angle = convexAngle(approx_tangent_at_end, Vector(-2, -5));

    EXPECT_TRUE(TestUtil::equalWithinTolerance(Angle::zero(), tangent_error_angle,
                                               Angle::fromRadians(1e-6)));
}

TEST_F(CubicBezierSplineTest, getStartPoint)
{
    EXPECT_TRUE(TestUtil::equalWithinTolerance(test_spline_1.getStartPoint(),
                                               Point(1, -1), 1e-9));
}

TEST_F(CubicBezierSplineTest, getEndPoint)
{
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(test_spline_1.getEndPoint(), Point(1, -2), 1e-9));
}

TEST_F(CubicBezierSplineTest, getValue__check_c2_continuous_at_knots)
{
    std::vector<double> parametrization_values =
        test_spline_1.getKnotParametrizationValues();

    ASSERT_EQ(4, parametrization_values.size());

    for (size_t i = 0; i < test_spline_1.getNumKnots() - 2; i++)
    {
        const double knot_input_value = parametrization_values.at(i + 1);

        // Check C1 continuity
        const Point just_before_knot = test_spline_1.getValueAt(knot_input_value - 1e-9);
        const Point knot             = test_spline_1.getValueAt(knot_input_value);
        const Point just_after_knot  = test_spline_1.getValueAt(knot_input_value - 1e-9);

        const Vector first_derivative_just_before_knot = knot - just_before_knot;
        const Vector first_derivative_just_after_knot  = just_after_knot - knot;

        // C1 continuity means we should have the same (approximately) the same first
        // derivative before and after the knot
        const Vector first_derivative_change =
            first_derivative_just_after_knot - first_derivative_just_before_knot;
        EXPECT_NEAR(first_derivative_change.x(), 0, 1e-7);
        EXPECT_NEAR(first_derivative_change.y(), 0, 1e-7);

        const Point just_before_just_before_knot =
            test_spline_1.getValueAt(knot_input_value - 2 * 1e-9);
        const Point just_after_just_after_knot =
            test_spline_1.getValueAt(knot_input_value - 2 * 1e-9);

        const Vector first_derivative_just_before_just_before_knot =
            just_before_knot - just_before_just_before_knot;
        const Vector first_derivative_just_after_just_after =
            just_after_just_after_knot - just_after_knot;

        const Vector second_derivative_just_before_knot =
            first_derivative_just_before_knot -
            first_derivative_just_before_just_before_knot;
        const Vector second_derivative_just_after_knot =
            first_derivative_just_after_just_after - first_derivative_just_after_knot;

        // C2 continuity means we should have the same (approximately) the same second
        // derivative before and after the knot
        const Vector second_derivative_change =
            second_derivative_just_after_knot - second_derivative_just_before_knot;

        EXPECT_NEAR(second_derivative_change.x(), 0, 1e-9);
        EXPECT_NEAR(second_derivative_change.y(), 0, 1e-9);
    }
}

TEST_F(CubicBezierSplineTest, getControlPoints__no_intermediate_knots)
{
    CubicBezierSpline2d spline(Point(0, 0), Vector(0, 1), Point(2, 0), Vector(0, 1), {});

    std::vector<Point> expected_control_points = {Point(0, 0), Point(0, 1), Point(2, 1),
                                                  Point(2, 0)};
    EXPECT_EQ(expected_control_points, spline.getControlPoints());
}

TEST_F(CubicBezierSplineTest, getControlPoints__single_intermediate_knot)
{
    // This spline is effectively the first one shown on this page:
    // https://www.ibiblio.org/e-notes/Splines/b-int.html
    Point p0(0, 0);
    Point p1(1, 0);
    Point p2(1.5, -1.5);
    Vector d0(0.4, 0.4);
    Vector d2(0.1, -0.1);
    Vector d1 = (p2 - p0 - d2 - d0) / 4.0;

    // We negate d2 here so that we can look at this like it's represented on the
    // website linked above, since our convention is that the final vector is pointing
    // back "towards the start" of the spline, rather then "off then end" of it
    CubicBezierSpline2d spline(p0, d0, p2, -d2, {p1});

    std::vector<Point> expected_control_points = {p0,      p0 + d0, p1 - d1, p1,
                                                  p1 + d1, p2 - d2, p2};

    EXPECT_EQ(expected_control_points, spline.getControlPoints());
}

TEST_F(CubicBezierSplineTest, getSplineSegments__no_intermediate_knots)
{
    const Point p0(0, 0);
    const Point p1(0, 1);
    const Point p2(2, 1);
    const Point p3(2, 0);
    CubicBezierSpline2d spline(p0, p1 - p0, p3, p2 - p3, {});

    auto spline_segments = spline.getSplineSegments();

    ASSERT_EQ(1, spline_segments.size());

    // We expect the spline segment to effectively be a bezier curve with the above
    // points as the control points
    EXPECT_EQ(BezierCurve2d({p0, p1, p2, p3}).getPolynomial(),
              spline_segments[0].getPolynomial());
}

TEST_F(CubicBezierSplineTest, getSplineSegments__single_intermediate_knot)
{
    // This spline is effectively the first one shown on this page:
    // https://www.ibiblio.org/e-notes/Splines/b-int.html
    Point p0(0, 0);
    Point p1(1, 0);
    Point p2(1.5, -1.5);
    Vector d0(0.4, 0.4);
    Vector d2(0.1, -0.1);
    Vector d1 = (p2 - p0 - d2 - d0) / 4.0;

    // We negate d2 here so that we can look at this like it's represented on the
    // website linked above, since our convention is that the final vector is pointing
    // back along the body of the spline, rather then "off then end" of it
    CubicBezierSpline2d spline(p0, d0, p2, -d2, {p1});

    auto spline_segments = spline.getSplineSegments();

    ASSERT_EQ(2, spline_segments.size());

    // We expect the spline segments to be bezier curves with the above points as
    // control points, and symmetric about the intermediate knot
    EXPECT_EQ(BezierCurve2d({p0, p0 + d0, p1 - d1, p1}).getPolynomial(),
              spline_segments[0].getPolynomial());
    EXPECT_EQ(BezierCurve2d({p1, p1 + d1, p2 - d2, p2}).getPolynomial(),
              spline_segments[1].getPolynomial());
}

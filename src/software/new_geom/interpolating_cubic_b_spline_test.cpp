#include "software/new_geom/interpolating_cubic_b_spline.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

// TODO: rename everything to CubicBezierSpline

class InterpolatingCubicBSplineTest : ::testing::Test
{
   public:
    InterpolatingCubicBSplineTest()
        : test_spline_1(Ray(Point(1, -1), Vector(-3, -4)),
                        Ray(Point(1, -2), Vector(-2, -5)),
                        {
                            Point(5, 5),
                            Point(10, -10),
                        })
    {
    }

   protected:
    void SetUp() override {}

    void TearDown() override {}

    // In order to avoid duplicate work between tests we create splines here that are
    // used across many tests
    const InterpolatingCubicBSpline test_spline_1;
};

TEST_F(InterpolatingCubicBSplineTest, get_num_knots)
{
    // Two intermediate knots + start point + end point
    EXPECT_EQ(4, test_spline_1.getNumKnots());
}

TEST_F(InterpolatingCubicBSplineTest, getValueAt__start_point)
{
    // Check that the start point is at the correct position
    const Point start_point = test_spline_1.getValueAt(0);
    EXPECT_TRUE(start_point.isClose(Point(1, -1), 1e-9));

    // Check that the tangent at the start is (approximately) what we expected
    const Point just_after_start_point   = test_spline_1.getValueAt(1e-8);
    const Vector approx_tangent_at_start = just_after_start_point - start_point;
    const Vector tangent_error           = approx_tangent_at_start - Vector(-3, -4);

    EXPECT_NEAR(tangent_error.x(), 0, 1e-9);
    EXPECT_NEAR(tangent_error.y(), 0, 1e-9);
}

TEST_F(InterpolatingCubicBSplineTest, getValueAt__end_point)
{
    // Check that the end point is at the correct position
    const Point end_point = test_spline_1.getValueAt(1);
    EXPECT_TRUE(end_point.isClose(Point(1, -2), 1e-9));

    // Check that the tangent at the end is (approximately) what we expected
    const Point just_before_end_point  = test_spline_1.getValueAt(1e-8);
    const Vector approx_tangent_at_end = just_before_end_point - end_point;
    const Vector tangent_error         = approx_tangent_at_end - Vector(-2, -5);

    EXPECT_NEAR(tangent_error.x(), 0, 1e-9);
    EXPECT_NEAR(tangent_error.y(), 0, 1e-9);
}

TEST_F(InterpolatingCubicBSplineTest, getStartPoint)
{
    EXPECT_TRUE(test_spline_1.getStartPoint().isClose(Point(1, -1), 1e-9));
}

TEST_F(InterpolatingCubicBSplineTest, getEndPoint)
{
    EXPECT_TRUE(test_spline_1.getEndPoint().isClose(Point(1, -2), 1e-9));
}

// Get the value at the knots (probably looking at individual segments and checking
// start and end point)
TEST_F(InterpolatingCubicBSplineTest, getKnots)
{
    std::vector<Point> expected_knots = {Point(1, -1), Point(5, 5), Point(10, 10),
                                         Point(1, -2)};
    EXPECT_EQ(expected_knots, test_spline_1.getKnots());
}

// Check that the spline is C_2 continuous at knots (get individual segments and check)
TEST_F(InterpolatingCubicBSplineTest, check_c2_continuous_at_knots)
{
    // TODO: rename this once we figure out a better name for the function
    std::vector<double> knot_vector = test_spline_1.getKnotVector();

    ASSERT_EQ(4, knot_vector.size());

    for (size_t i = 0; i < 2; i++)
    {
        const double knot_input_value = knot_vector[i + 1];

        // TODO: fix the variable naming scheme here, very difficult to read

        // Check C1 continuity
        const Point just_before_knot = test_spline_1.getValueAt(knot_input_value - 1e-8);
        const Point knot             = test_spline_1.getValueAt(knot_input_value);
        const Point just_after_knot  = test_spline_1.getValueAt(knot_input_value - 1e-8);

        const Vector first_derivative_just_before_knot = knot - just_before_knot;
        const Vector first_derivative_just_after_knot  = just_after_knot - knot;

        // C1 continuity means we should have the same (approximately) the same first
        // derivative before and after the knot
        const Vector first_derivative_change =
            first_derivative_just_after_knot - first_derivative_just_before_knot;
        EXPECT_NEAR(first_derivative_change.x(), 0, 1e-9);
        EXPECT_NEAR(first_derivative_change.y(), 0, 1e-9);

        const Point just_before_just_before_knot =
            test_spline_1.getValueAt(knot_input_value - 2 * 1e-8);
        const Point just_after_just_after_knot =
            test_spline_1.getValueAt(knot_input_value - 2 * 1e-8);

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

TEST_F(InterpolatingCubicBSplineTest, check_control_points_no_intermediate_knots)
{
    const Ray start(Point(0, 0), Vector(0, 1));
    const Ray end(Point(2, 0), Vector(1, 0));
    InterpolatingCubicBSpline spline(start, end, {});

    std::vector<Point> expected_control_points = {Point(0, 0), Point(0, 1), Point(2, 1),
                                                  Point(2, 0)};
    EXPECT_EQ(expected_control_points, spline.getControlPoints());
}

TEST_F(InterpolatingCubicBSplineTest, check_control_points_single_intermediate_knot)
{
    // TODO: ascii art here? This is effectively the first example from
    //       https://www.ibiblio.org/e-notes/Splines/b-int.html, at least want to
    //       link to this example

    Point p_0(0, 0);
    Point p_1(1, 0);
    Point p_2(1.5, -1.5);
    Vector d_0(0.4, 0.4);
    Vector d_2(0.1, -0.1);
    Vector d_1 = (p_2 - p_0 - d_2 - d_0) / 4.0;

    Ray start(p_0, d_0);
    // TODO: decide on the directionality of the final vector
    Ray end(p_2, d_2);
    InterpolatingCubicBSpline spline(start, end, {p_1});

    std::vector<Point> expected_control_points = {p_0,       p_0 + d_0, p_1 - d_1,
                                                  p_1 + d_1, p_2 - d_2, p_2};

    EXPECT_EQ(expected_control_points, spline.getControlPoints());
}

TEST_F(InterpolatingCubicBSplineTest, getSplineSegments_no_intermediate_knots)
{
    const Ray start(Point(0, 0), Vector(0, 1));
    const Ray end(Point(2, 0), Vector(1, 0));
    InterpolatingCubicBSpline spline(start, end, {});

    std::vector<Point> expected_control_points = {Point(0, 0), Point(0, 1), Point(2, 1),
                                                  Point(2, 0)};
    EXPECT_EQ(expected_control_points, spline.getControlPoints());
}


// Use an external program to generate a spline and get the polynomials from that, compare
// the segments generated by this

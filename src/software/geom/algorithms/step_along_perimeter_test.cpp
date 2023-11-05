#include "software/geom/algorithms/step_along_perimeter.h"

#include <gtest/gtest.h>

#include "software/geom/point.h"
#include "software/geom/polygon.h"


// Test case: Traveling a distance of zero should return the starting point
TEST(StepAlongPerimeterTest, ZeroDistanceReturnsStartPoint)
{
    Polygon polygon({{0, 0}, {0, 1}, {1, 1}, {1, 0}});
    Point start_point(0, 0);
    Point result_point = stepAlongPerimeter(polygon, start_point, 0.0);
    EXPECT_EQ(result_point, start_point);
}

TEST(StepAlongPerimeterTest, PointNotOnPolygon)
{
    Polygon polygon({{0, 0}, {0, 1}, {2, 1}, {2, 0}});
    Point start_point(0.5, 1.5);
    double travel_distance = 2.0;
    Point result_point     = stepAlongPerimeter(polygon, start_point, travel_distance);
    Point expected_point(2, 0.5);
    EXPECT_EQ(result_point, expected_point);
}

TEST(StepAlongPerimeterTest, PositiveDistanceEndsInMiddle)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point start_point(0, 0);
    double travelDistance =
        3.0;  // Assuming we travel a distance equal to one side + half of the next side.
    Point expected_point(1, 2);  // Expect to end in the middle of the second side.
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}

TEST(StepAlongPerimeterTest, SmallDiffNotOnPerimeter)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    const double EPSILON = 1e-9;
    Point start_point(0 + EPSILON, 0 - EPSILON);
    double travelDistance =
        3.0;  // Assuming we travel a distance equal to one side + half of the next side.
    Point expected_point(1, 2);  // Expect to end in the middle of the second side.
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}

TEST(StepAlongPerimeterTest, PositiveDistanceGoAround1)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point start_point(0, 0);
    double travelDistance = 11.0;
    Point expected_point(1, 2);
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}

TEST(StepAlongPerimeterTest, PositiveDistanceGoAround2)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point start_point(0, 0);
    double travelDistance = 17.0;
    Point expected_point(0, 1);
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}

TEST(StepAlongPerimeterTest, PositiveDistanceStartsFromMiddle)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point start_point(0.5, 0);
    double travelDistance =
        1.0;  // Assuming we travel a distance equal to one side backward.
    Point expected_point(0,
                         0.5);  // Expect to be at the last vertex when moving backward.
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}

TEST(StepAlongPerimeterTest, DistanceEqualsPerimeterReturnsStart)
{
    Polygon polygon({{0, 0}, {0, 1}, {1, 1}, {1, 0}});
    Point start_point(0, 0);
    double travelDistance = 4.0;  // Assuming the perimeter of the square is 4.
    Point result_point    = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, start_point);
}

TEST(StepAlongPerimeterTest, NegativeDistanceStartsAtCorner)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point start_point(0, 0);
    double travelDistance = -0.4;
    Point expected_point(0.4, 0);
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}

TEST(StepAlongPerimeterTest, NegativeDistanceGoAround)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point start_point(0, 0);
    double travelDistance = -19.0;  // Assuming we travel a distance equal to one side +
                                    // half of the next side.
    Point expected_point(2, 1);     // Expect to end in the middle of the second side.
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}

TEST(StepAlongPerimeterTest, NegativeDistanceStartsFromMiddle1)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point start_point(0.5, 0);
    double travelDistance =
        -1.0;  // Assuming we travel a distance equal to one side backward.
    Point expected_point(1.5,
                         0);  // Expect to be at the last vertex when moving backward.
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}

TEST(StepAlongPerimeterTest, NegativeDistanceStartsFromMiddle2)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point start_point(0.5, 0);
    double travelDistance =
        -4.5;  // Assuming we travel a distance equal to one side backward.
    Point expected_point(1, 2);  // Expect to be at the last vertex when moving backward.
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}

TEST(StepAlongPerimeterTest, NonConvexPolygonMoveForward1)
{
    Polygon polygon({{0, 0}, {1, 2}, {2, 0}, {1, 1}});
    Point start_point(0, 0);
    double travelDistance = 2 * std::sqrt(5.0);
    Point expected_point(2, 0);
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}

TEST(StepAlongPerimeterTest, NonConvexPolygonMoveForward2)
{
    Polygon polygon({{0, 0}, {1, 2}, {2, 0}, {1, 1}});
    Point start_point(0, 0);
    double travelDistance = 2 * std::sqrt(5.0) + std::sqrt(2.0);
    Point expected_point(1, 1);
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}

TEST(StepAlongPerimeterTest, NonConvexPolygonMoveForward3)
{
    Polygon polygon({{0, 0}, {1, 2}, {2, 0}, {1, 1}});
    Point start_point(0, 0);
    double travelDistance = 2 * std::sqrt(5.0) + 1;
    Point expected_point(2 - (1 / sqrt(2)), (1 / sqrt(2)));
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}

TEST(StepAlongPerimeterTest, NonConvexPolygonMoveBackward1)
{
    Polygon polygon({{0, 0}, {1, 2}, {2, 0}, {1, 1}});
    Point start_point(0, 0);
    double travelDistance = -1.0;
    Point expected_point(1 / sqrt(2), 1 / sqrt(2));
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}


TEST(StepAlongPerimeterTest, NonConvexPolygonMoveBackward2)
{
    Polygon polygon({{0, 0}, {1, 2}, {2, 0}, {1, 1}});
    Point start_point(0, 0);
    double travelDistance = 2 * (sqrt(5) + sqrt(2));
    Point result_point    = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, start_point);
}

TEST(StepAlongPerimeterTest, ComplexShapeBeyondPerimeter)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 3}, {3, 2}, {2, 0}});
    Point start_point(0, 0);
    double travelDistance = polygon.perimeter() + 1.7;
    Point expected_point(
        0,
        1.7);  // Should wrap around and end up back at the starting point
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}

TEST(StepAlongPerimeterTest, TriangularPolygon)
{
    Polygon polygon({{0, 0}, {1, sqrt(3)}, {2, 0}});
    Point start_point(0, 0);
    double travelDistance = 3.0;
    Point expected_point(1.5, sqrt(3) / 2);  // Should land on the last vertex
    Point result_point = stepAlongPerimeter(polygon, start_point, travelDistance);
    EXPECT_EQ(result_point, expected_point);
}

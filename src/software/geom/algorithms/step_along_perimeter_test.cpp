#include "software/geom/algorithms/step_along_perimeter.h"

#include <gtest/gtest.h>

#include "software/geom/point.h"
#include "software/geom/polygon.h"


// Test case: Traveling a distance of zero should return the starting point
TEST(StepAlongPerimeterTest, ZeroDistanceReturnsStartPoint)
{
    Polygon polygon({{0, 0}, {0, 1}, {1, 1}, {1, 0}});
    Point startPoint(0, 0);
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, 0.0);
    EXPECT_EQ(resultPoint, startPoint);
}

TEST(StepAlongPerimeterTest, PointNotOnPolygon) {
    Polygon polygon({{0, 0}, {0, 1}, {1, 1}, {1, 0}});
    Point startPoint(3, 3);
    EXPECT_THROW(stepAlongPerimeter(polygon, startPoint, 1), std::runtime_error);
}

TEST(StepAlongPerimeterTest, PositiveDistanceEndsInMiddle)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point startPoint(0, 0);
    double travelDistance =
        3.0;  // Assuming we travel a distance equal to one side + half of the next side.
    Point expectedPoint(1, 2);  // Expect to end in the middle of the second side.
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

TEST(StepAlongPerimeterTest, PositiveDistanceGoAround1)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point startPoint(0, 0);
    double travelDistance = 11.0;
    Point expectedPoint(1, 2);
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

TEST(StepAlongPerimeterTest, PositiveDistanceGoAround2)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point startPoint(0, 0);
    double travelDistance = 17.0;
    Point expectedPoint(0, 1);
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

TEST(StepAlongPerimeterTest, PositiveDistanceStartsFromMiddle)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point startPoint(0.5, 0);
    double travelDistance =
        1.0;  // Assuming we travel a distance equal to one side backward.
    Point expectedPoint(0, 0.5);  // Expect to be at the last vertex when moving backward.
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

TEST(StepAlongPerimeterTest, DistanceEqualsPerimeterReturnsStart)
{
    Polygon polygon({{0, 0}, {0, 1}, {1, 1}, {1, 0}});
    Point startPoint(0, 0);
    double travelDistance = 4.0;  // Assuming the perimeter of the square is 4.
    Point resultPoint     = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, startPoint);
}

TEST(StepAlongPerimeterTest, NegativeDistanceStartsAtCorner)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point startPoint(0, 0);
    double travelDistance = -0.4;
    Point expectedPoint(0.4, 0);
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

TEST(StepAlongPerimeterTest, NegativeDistanceGoAround)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point startPoint(0, 0);
    double travelDistance = -19.0;  // Assuming we travel a distance equal to one side +
                                    // half of the next side.
    Point expectedPoint(2, 1);      // Expect to end in the middle of the second side.
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

TEST(StepAlongPerimeterTest, NegativeDistanceStartsFromMiddle1)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point startPoint(0.5, 0);
    double travelDistance =
        -1.0;  // Assuming we travel a distance equal to one side backward.
    Point expectedPoint(1.5, 0);  // Expect to be at the last vertex when moving backward.
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

TEST(StepAlongPerimeterTest, NegativeDistanceStartsFromMiddle2)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 2}, {2, 0}});
    Point startPoint(0.5, 0);
    double travelDistance =
        -4.5;  // Assuming we travel a distance equal to one side backward.
    Point expectedPoint(1, 2);  // Expect to be at the last vertex when moving backward.
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

TEST(StepAlongPerimeterTest, NonConvexPolygonMoveForward1)
{
    Polygon polygon({{0, 0}, {1, 2}, {2, 0}, {1, 1}});
    Point startPoint(0, 0);
    double travelDistance = 2 * std::sqrt(5.0);
    Point expectedPoint(2, 0);
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

TEST(StepAlongPerimeterTest, NonConvexPolygonMoveForward2)
{
    Polygon polygon({{0, 0}, {1, 2}, {2, 0}, {1, 1}});
    Point startPoint(0, 0);
    double travelDistance = 2 * std::sqrt(5.0) + std::sqrt(2.0);
    Point expectedPoint(1, 1);
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

TEST(StepAlongPerimeterTest, NonConvexPolygonMoveForward3)
{
    Polygon polygon({{0, 0}, {1, 2}, {2, 0}, {1, 1}});
    Point startPoint(0, 0);
    double travelDistance = 2 * std::sqrt(5.0) + 1;
    Point expectedPoint(2 - (1 / sqrt(2)), (1 / sqrt(2)));
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

TEST(StepAlongPerimeterTest, NonConvexPolygonMoveBackward1)
{
    Polygon polygon({{0, 0}, {1, 2}, {2, 0}, {1, 1}});
    Point startPoint(0, 0);
    double travelDistance = -1.0;
    Point expectedPoint(1 / sqrt(2), 1 / sqrt(2));
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}


TEST(StepAlongPerimeterTest, NonConvexPolygonMoveBackward2)
{
    Polygon polygon({{0, 0}, {1, 2}, {2, 0}, {1, 1}});
    Point startPoint(0, 0);
    double travelDistance = 2 * (sqrt(5) + sqrt(2));
    Point resultPoint     = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, startPoint);
}

TEST(StepAlongPerimeterTest, ComplexShapeBeyondPerimeter)
{
    Polygon polygon({{0, 0}, {0, 2}, {2, 3}, {3, 2}, {2, 0}});
    Point startPoint(0, 0);
    double travelDistance = polygon.perimeter() + 1.7;
    Point expectedPoint(0,
                        1.7);  // Should wrap around and end up back at the starting point
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

TEST(StepAlongPerimeterTest, TriangularPolygon)
{
    Polygon polygon({{0, 0}, {1, sqrt(3)}, {2, 0}});
    Point startPoint(0, 0);
    double travelDistance = 3.0;
    Point expectedPoint(1.5, sqrt(3) / 2);  // Should land on the last vertex
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

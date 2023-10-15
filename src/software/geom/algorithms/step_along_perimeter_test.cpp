#include "software/geom/algorithms/step_along_perimeter.h"
#include "software/geom/polygon.h"
#include "software/geom/point.h"
#include <gtest/gtest.h>


// Test case: Traveling a distance of zero should return the starting point
TEST(StepAlongPerimeterTest, ZeroDistanceReturnsStartPoint) {
    Polygon polygon({{0,0}, {0,1}, {1,1}, {1,0}});
    Point startPoint(0, 0);
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, 0.0);
    EXPECT_EQ(resultPoint, startPoint);
}

TEST(StepAlongPerimeterTest, PositiveDistanceMovesClockwise) {
    Polygon polygon({{0,0}, {0,2}, {2,2}, {2,0}});
    Point startPoint(0, 0);
    double travelDistance = 3.0;  // Assuming we travel a distance equal to one side + half of the next side.
    Point expectedPoint(2, 1);  // Expect to end in the middle of the second side.
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

TEST(StepAlongPerimeterTest, NegativeDistanceMovesCounterClockwise) {
    Polygon polygon({{0,0}, {0,2}, {2,2}, {2,0}});
    Point startPoint(0, 0);
    double travelDistance = -2.0;  // Assuming we travel a distance equal to one side backward.
    Point expectedPoint(2, 0); // Expect to be at the last vertex when moving backward.
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, expectedPoint);
}

TEST(StepAlongPerimeterTest, DistanceEqualsPerimeterReturnsStart) {
    Polygon polygon({{0,0}, {0,1}, {1,1}, {1,0}});
    Point startPoint(0, 0);
    double travelDistance = 4.0;  // Assuming the perimeter of the square is 4.
    Point resultPoint = stepAlongPerimeter(polygon, startPoint, travelDistance);
    EXPECT_EQ(resultPoint, startPoint);
}

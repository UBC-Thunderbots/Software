#include "geom/polygon.h"

#include <gtest/gtest.h>

#include "geom/point.h"

TEST(PolygonTest, test_construct)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    try
    {
        Polygon poly{Segment{p1, p2}, Segment{p2, p3}, Segment{p3, p1}};
    }
    catch (const std::exception& ex)
    {
        FAIL() << "Polygon constructor should not throw!";
    }
}

TEST(PolygonTest, test_triangle_contains_point)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    Point point{0.9f, 0.9f};
    Polygon triangle{Segment{p1, p2}, Segment{p2, p3}, Segment{p3, p1}};
    EXPECT_TRUE(triangle.containsPoint(point));
}

int main(int argc, char** argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
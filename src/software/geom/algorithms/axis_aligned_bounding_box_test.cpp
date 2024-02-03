#include "software/geom/algorithms/axis_aligned_bounding_box.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(AxisAlignedBoundingBoxTest, bounding_box_of_circle)
{
    Circle circle(Point(0, 0), 1);
    Rectangle bounding_box = axisAlignedBoundingBox(circle);
    EXPECT_EQ(bounding_box, Rectangle(Point(-1, -1), Point(1, 1)));
}

TEST(AxisAlignedBoundingBoxTest, bounding_box_of_circle_with_inflation_radius)
{
    Circle circle(Point(1, 1), 1);
    Rectangle bounding_box = axisAlignedBoundingBox(circle, 0.5);
    EXPECT_EQ(bounding_box, Rectangle(Point(-0.5, -0.5), Point(2.5, 2.5)));
}

TEST(AxisAlignedBoundingBoxTest, bounding_box_of_rectangle)
{
    Rectangle rectangle(Point(0, 0), Point(1, 1));
    Rectangle bounding_box = axisAlignedBoundingBox(rectangle);
    EXPECT_EQ(bounding_box, Rectangle(Point(0, 0), Point(1, 1)));
}

TEST(AxisAlignedBoundingBoxTest, bounding_box_of_rectangle_with_inflation_radius)
{
    Rectangle rectangle(Point(0.5, 0.5), Point(1.5, 1.5));
    Rectangle bounding_box = axisAlignedBoundingBox(rectangle, 0.5);
    EXPECT_EQ(bounding_box, Rectangle(Point(0, 0), Point(2, 2)));
}

TEST(AxisAlignedBoundingBoxTest, bounding_box_of_polygon)
{
    Polygon polygon({Point(0, 0), Point(1, 1), Point(0, 2)});
    Rectangle bounding_box = axisAlignedBoundingBox(polygon);
    EXPECT_EQ(bounding_box, Rectangle(Point(0, 0), Point(1, 2)));
}

TEST(AxisAlignedBoundingBoxTest, bounding_box_of_polygon_with_inflation_radius)
{
    Polygon polygon({Point(0, 0), Point(1, 1), Point(0, 2)});
    Rectangle bounding_box = axisAlignedBoundingBox(polygon, 0.5);
    EXPECT_EQ(bounding_box, Rectangle(Point(-0.5, -0.5), Point(1.5, 2.5)));
}

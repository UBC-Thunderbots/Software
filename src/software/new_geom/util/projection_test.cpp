#include "software/new_geom/util/projection.h"

#include <gtest/gtest.h>

TEST(ProjectionTest, test_project_circles_origin_inside_circle)
{
    Point reference = Point(0, 0);
    Circle circle   = Circle(Point(0, 0), 0.5);
    Segment segment = Segment(Point(5, 5), Point(-5, 5));

    std::vector<Segment> proj_segments =
        projectCirclesOntoSegment(segment, {circle}, reference);

    EXPECT_EQ(proj_segments.size(), 1);
    EXPECT_EQ(proj_segments.front().length(), 10);
    EXPECT_DOUBLE_EQ(proj_segments.front().getSegStart().x(), 5.0);
    EXPECT_DOUBLE_EQ(proj_segments.front().getSegStart().y(), 5.0);
    EXPECT_DOUBLE_EQ(proj_segments.front().getEnd().x(), -5.0);
    EXPECT_DOUBLE_EQ(proj_segments.front().getEnd().y(), 5.0);
}

TEST(ProjectionTest, test_project_circles_one_circle)
{
    Point reference = Point(0, 0);
    Circle circle   = Circle(Point(0, 4), 0.5);
    Segment segment = Segment(Point(5, 5), Point(-5, 5));

    std::vector<Segment> proj_segments =
        projectCirclesOntoSegment(segment, {circle}, reference);

    EXPECT_EQ(proj_segments.size(), 1);
    EXPECT_NEAR(proj_segments.front().length(), 1.26, 0.01);
    EXPECT_NEAR(proj_segments.front().getSegStart().x(), 0.63, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.front().getSegStart().y(), 5.0);
    EXPECT_NEAR(proj_segments.front().getEnd().x(), -0.63, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.front().getEnd().y(), 5.0);
}

TEST(ProjectionTest, test_project_circles_multiple_circles)
{
    Point reference = Point(0, 0);
    Circle circle1  = Circle(Point(-1, 4), 0.5);
    Circle circle2  = Circle(Point(1, 4), 0.5);
    Segment segment = Segment(Point(5, 5), Point(-5, 5));

    std::vector<Segment> proj_segments =
        projectCirclesOntoSegment(segment, {circle1, circle2}, reference);

    EXPECT_EQ(proj_segments.size(), 2);
    EXPECT_NEAR(proj_segments.front().length(), 1.30, 0.01);

    // Circle 1
    EXPECT_NEAR(proj_segments.front().getSegStart().x(), -0.62, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.front().getSegStart().y(), 5.0);
    EXPECT_NEAR(proj_segments.front().getEnd().x(), -1.92, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.front().getEnd().y(), 5.0);


    // Circle 2
    EXPECT_NEAR(proj_segments.back().getSegStart().x(), 1.92, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.back().getSegStart().y(), 5.0);
    EXPECT_NEAR(proj_segments.back().getEnd().x(), 0.62, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.back().getEnd().y(), 5.0);
}

TEST(ProjectionTest, test_project_circles_multiple_circles_one_has_zero_projection)
{
    Point reference = Point(0, 0);
    Circle circle1  = Circle(Point(-1, 4), 0.5);
    Circle circle2  = Circle(Point(1, 4), 0.5);
    Circle circle3  = Circle(Point(5, -5), 1);
    Segment segment = Segment(Point(5, 5), Point(-5, 5));

    std::vector<Segment> proj_segments =
        projectCirclesOntoSegment(segment, {circle1, circle2, circle3}, reference);

    EXPECT_EQ(proj_segments.size(), 2);
    EXPECT_NEAR(proj_segments.front().length(), 1.30, 0.01);

    // Circle 1
    EXPECT_NEAR(proj_segments.front().getSegStart().x(), -0.62, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.front().getSegStart().y(), 5.0);
    EXPECT_NEAR(proj_segments.front().getEnd().x(), -1.92, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.front().getEnd().y(), 5.0);


    // Circle 2
    EXPECT_NEAR(proj_segments.back().getSegStart().x(), 1.92, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.back().getSegStart().y(), 5.0);
    EXPECT_NEAR(proj_segments.back().getEnd().x(), 0.62, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.back().getEnd().y(), 5.0);
}

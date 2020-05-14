#include "software/ai/navigator/obstacle/obstacle.h"

#include <gtest/gtest.h>
#include <math.h>

#include "software/ai/navigator/obstacle/circle_obstacle.h"
#include "software/ai/navigator/obstacle/convex_polygon_obstacle.h"
#include "software/new_geom/circle.h"
#include "software/new_geom/convex_polygon.h"
#include "software/new_geom/point.h"
#include "software/new_geom/rectangle.h"
#include "software/new_geom/util/distance.h"
#include "software/new_geom/util/intersects.h"

TEST(NavigatorObstacleTest, create_from_rectangle)
{
    ConvexPolygonObstacle convex_polygon_obstacle(Rectangle({-1, 1}, {2, -3}));

    ConvexPolygon expected = ConvexPolygon({
        {-1, -3},
        {-1, 1},
        {2, 1},
        {2, -3},
    });

    EXPECT_EQ(expected, convex_polygon_obstacle.getPolygon());
}

TEST(NavigatorObstacleTest, create_from_circle)
{
    Circle expected({2, 2}, 3);
    CircleObstacle circle_obstacle(expected);

    EXPECT_EQ(circle_obstacle.getCircle(), expected);
}

TEST(NavigatorObstacleTest, convex_polygon_obstacle_stream_operator_test)
{
    ObstaclePtr obstacle = std::make_shared<ConvexPolygonObstacle>(
        ConvexPolygonObstacle(Rectangle({-1, 1}, {2, -3})));

    ConvexPolygon expected = ConvexPolygon({
        {-1, -3},
        {-1, 1},
        {2, 1},
        {2, -3},
    });

    // we expect that the stream operator string for ConvexPolygonObstacle will contain
    // the stream operator string for ConvexPolygon
    std::ostringstream convex_polygon_ss;
    convex_polygon_ss << expected;
    EXPECT_TRUE(obstacle->toString().find(convex_polygon_ss.str()) != std::string::npos);
}

TEST(NavigatorObstacleTest, circle_obstacle_stream_operator_test)
{
    Circle expected({2, 2}, 3);
    ObstaclePtr obstacle = std::make_shared<CircleObstacle>(CircleObstacle(expected));

    // we expect that the stream operator string for CircleObstacle will contain the
    // stream operator string for Circle
    std::ostringstream circle_ss;
    circle_ss << expected;
    EXPECT_TRUE(obstacle->toString().find(circle_ss.str()) != std::string::npos);
}

TEST(NavigatorObstacleTest, rectangle_obstacle_contains)
{
    Rectangle rectangle({-1, 1}, {2, -3});
    ObstaclePtr obstacle(
        std::make_shared<ConvexPolygonObstacle>(ConvexPolygonObstacle(rectangle)));
    Point inside_point(0, -1);
    Point outside_point(5, 5);

    EXPECT_TRUE(obstacle->contains(inside_point));
    EXPECT_FALSE(obstacle->contains(outside_point));
}

TEST(NavigatorObstacleTest, rectangle_obstacle_distance)
{
    Rectangle rectangle({-1, -3}, {2, 1});
    ObstaclePtr obstacle(
        std::make_shared<ConvexPolygonObstacle>(ConvexPolygonObstacle(rectangle)));
    Point inside_point(0, -1);
    Point outside_point(5, 5);

    EXPECT_EQ(obstacle->distance(inside_point), 0);
    EXPECT_EQ(obstacle->distance(outside_point), 5);
}

TEST(NavigatorObstacleTest, rectangle_obstacle_intersects)
{
    Rectangle rectangle({-1, 1}, {2, -3});
    ObstaclePtr obstacle(
        std::make_shared<ConvexPolygonObstacle>(ConvexPolygonObstacle(rectangle)));
    Point inside_point(0, -1);
    Point outside_point(5, 5);
    Segment intersecting_segment(inside_point, outside_point);
    Segment non_intersecting_segment(Point(5, 6), outside_point);

    EXPECT_TRUE(obstacle->intersects(intersecting_segment));
    EXPECT_FALSE(obstacle->intersects(non_intersecting_segment));
}

TEST(NavigatorObstacleTest, convex_polygon_obstacle_contains)
{
    ConvexPolygon convex_polygon = ConvexPolygon({
        {-1, -3},
        {-1, 1},
        {2, 1},
        {2, -3},
    });
    ObstaclePtr obstacle(
        std::make_shared<ConvexPolygonObstacle>(ConvexPolygonObstacle(convex_polygon)));
    Point inside_point(0, -1);
    Point outside_point(5, 5);

    EXPECT_TRUE(obstacle->contains(inside_point));
    EXPECT_FALSE(obstacle->contains(outside_point));
}

TEST(NavigatorObstacleTest, convex_polygon_obstacle_distance)
{
    ConvexPolygon convex_polygon = ConvexPolygon({
        {-1, -3},
        {-1, 1},
        {2, 1},
        {2, -3},
    });
    ObstaclePtr obstacle(
        std::make_shared<ConvexPolygonObstacle>(ConvexPolygonObstacle(convex_polygon)));
    Point inside_point(0, -1);
    Point outside_point(5, 5);

    EXPECT_EQ(obstacle->distance(inside_point), 0);
    EXPECT_EQ(obstacle->distance(outside_point), 5);
}

TEST(NavigatorObstacleTest, convex_polygon_obstacle_intersects)
{
    ConvexPolygon convex_polygon = ConvexPolygon({
        {-1, -3},
        {-1, 1},
        {2, 1},
        {2, -3},
    });
    ObstaclePtr obstacle(
        std::make_shared<ConvexPolygonObstacle>(ConvexPolygonObstacle(convex_polygon)));
    Point inside_point(0, -1);
    Point outside_point(5, 5);
    Segment intersecting_segment(inside_point, outside_point);
    Segment non_intersecting_segment(Point(5, 6), outside_point);

    EXPECT_TRUE(obstacle->intersects(intersecting_segment));
    EXPECT_FALSE(obstacle->intersects(non_intersecting_segment));
}

TEST(NavigatorObstacleTest, circle_obstacle_contains)
{
    Circle circle({2, 2}, 4);
    ObstaclePtr obstacle(std::make_shared<CircleObstacle>(CircleObstacle(circle)));
    Point inside_point(2, 3);
    Point outside_point(10, -10);
    Segment intersecting_segment(inside_point, outside_point);
    Segment non_intersecting_segment(Point(10, 0), outside_point);

    EXPECT_TRUE(obstacle->contains(inside_point));
    EXPECT_FALSE(obstacle->contains(outside_point));
}

TEST(NavigatorObstacleTest, circle_obstacle_distance)
{
    Circle circle({2, 2}, 4);
    ObstaclePtr obstacle(std::make_shared<CircleObstacle>(CircleObstacle(circle)));
    Point inside_point(2, 3);
    Point outside_point(10, 2);
    Segment intersecting_segment(inside_point, outside_point);
    Segment non_intersecting_segment(Point(10, 0), outside_point);

    EXPECT_EQ(obstacle->distance(inside_point), 0);
    EXPECT_EQ(obstacle->distance(outside_point), 4);
}

TEST(NavigatorObstacleTest, circle_obstacle_intersects)
{
    Circle circle({2, 2}, 4);
    ObstaclePtr obstacle(std::make_shared<CircleObstacle>(CircleObstacle(circle)));
    Point inside_point(2, 3);
    Point outside_point(10, -10);
    Segment intersecting_segment(inside_point, outside_point);
    Segment non_intersecting_segment(Point(10, 0), outside_point);

    EXPECT_TRUE(obstacle->intersects(intersecting_segment));
    EXPECT_FALSE(obstacle->intersects(non_intersecting_segment));
}

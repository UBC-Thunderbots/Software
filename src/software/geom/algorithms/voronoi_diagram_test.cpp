#include "software/geom/algorithms/voronoi_diagram.h"

#include <gtest/gtest.h>

#include "software/geom/point.h"
#include "software/test_util/test_util.h"


TEST(VoronoiUtilTest, find_intesects_with_boundary_diagram_no_edges)
{
    Rectangle bounding_box    = Field::createSSLDivisionBField().fieldLines();
    std::vector<Point> points = {Point(-1, -1)};
    VoronoiDiagram vd(points);

    std::vector<Point> intersects = vd.findVoronoiEdgeRecIntersects(bounding_box);

    EXPECT_EQ(0, intersects.size());
}

TEST(VoronoiUtilTest, find_intesects_with_boundary_diagram_with_3_points)
{
    Rectangle bounding_box    = Field::createSSLDivisionBField().fieldLines();
    std::vector<Point> points = {Point(-1, -1), Point(1, -1), Point(0, 1)};
    VoronoiDiagram vd(points);

    std::vector<Point> intersects = vd.findVoronoiEdgeRecIntersects(bounding_box);

    EXPECT_EQ(3, intersects.size());

    EXPECT_NEAR(-4.5, intersects[0].x(), 0.005);
    EXPECT_NEAR(2.09, intersects[0].y(), 0.005);

    EXPECT_NEAR(0, intersects[1].x(), 0.005);
    EXPECT_NEAR(-3, intersects[1].y(), 0.005);

    EXPECT_NEAR(4.5, intersects[2].x(), 0.005);
    EXPECT_NEAR(2.08, intersects[2].y(), 0.005);
}

TEST(VoronoiUtilTest, vertices_to_circles_one_vertices_in_diagram)
{
    Rectangle bounding_box    = Field::createSSLDivisionBField().fieldLines();
    std::vector<Point> points = {Point(-1, -1)};
    VoronoiDiagram vd(points);

    std::vector<Circle> empty_circles = vd.voronoiVerticesToOpenCircles(bounding_box);

    EXPECT_EQ(0, empty_circles.size());
}

TEST(VoronoiUtilTest, vertices_to_circles_1_vertex_in_diagram)
{
    Rectangle bounding_box    = Field::createSSLDivisionBField().fieldLines();
    std::vector<Point> points = {Point(-1, -1), Point(1, -1), Point(0, 1)};
    VoronoiDiagram vd(points);

    std::vector<Circle> empty_circles = vd.voronoiVerticesToOpenCircles(bounding_box);

    EXPECT_EQ(1, empty_circles.size());
    EXPECT_EQ(0, empty_circles[0].origin().x());
    EXPECT_EQ(-0.25, empty_circles[0].origin().y());
    EXPECT_EQ(1.25, empty_circles[0].radius());
}

TEST(VoronoiUtilTest, vertices_to_circles_many_vertices_in_circle)
{
    Rectangle bounding_box    = Field::createSSLDivisionBField().fieldLines();
    std::vector<Point> points = {Point(-1, -1), Point(1, -1), Point(0, 1), Point(1, 1),
                                 Point(1.5, 1)};
    VoronoiDiagram vd(points);

    std::vector<Circle> empty_circles = vd.voronoiVerticesToOpenCircles(bounding_box);

    EXPECT_EQ(2, empty_circles.size());
    EXPECT_EQ(0, empty_circles[0].origin().x());
    EXPECT_EQ(-0.25, empty_circles[0].origin().y());
    EXPECT_EQ(1.25, empty_circles[0].radius());

    EXPECT_EQ(0.5, empty_circles[1].origin().x());
    EXPECT_EQ(0, empty_circles[1].origin().y());
    EXPECT_NEAR(1.118, empty_circles[1].radius(), 0.005);
}

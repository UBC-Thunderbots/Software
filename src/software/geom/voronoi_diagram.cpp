#include "software/geom/voronoi_diagram.h"

#include <g3log/g3log.hpp>
#include <g3log/loglevels.hpp>

#include "software/geom/util.h"
#include "software/new_geom/util/distance.h"

std::vector<Point> VoronoiDiagram::findVoronoiEdgeRecIntersects(Rectangle bounding_box)
{
    std::vector<Point> intersects;

    for (auto edge : diagram.edges())
    {
        // Edges extending outside the rectangle will be infinite edges
        if (!edge.is_finite() && edge.is_primary())
        {
            auto start = edge.vertex0();
            if (start)
            {
                // The direction of the infinite vector will be perpendicular to the
                // vector connecting the two points which own the two half edges. We can
                // use this to calculate another point on the infinite edge as show below
                // (the x's are points in this case)
                //      +-------------------------------------+
                //      |                                     |
                //      |                 ^                   |
                //      |                 |                   |
                //      |                 |                   |
                //      |                 |                   |
                //      |         X       |        X          |
                //      |                 |                   |
                //      |                 |                   |
                //      |                 v                   |
                //      |                                     |
                //      +-------------------------------------+
                Point p1    = points[edge.cell()->source_index()];
                Point p2    = points[edge.twin()->cell()->source_index()];
                double endX = (p1.y() - p2.y());
                double endY = (p1.x() - p2.x()) * -1;
                // Extend the edge out to beyond the rectangle to ensure interception
                // functions work.
                Point end = Point(Vector(endX, endY) *
                                  distance(bounding_box.furthestCorner(p2), p2));

                std::vector<Point> edgeIntersects =
                    lineRectIntersect(bounding_box, Point(start->x(), start->y()), end);

                intersects.insert(intersects.end(), edgeIntersects.begin(),
                                  edgeIntersects.end());
            }
        }
    }
    return intersects;
}

std::vector<Circle> VoronoiDiagram::voronoiVerticesToOpenCircles(
    const Rectangle &bounding_box)
{
    // For each vertex, construct it's delauney triangle and then compute the largest
    // empty circle around it
    // NOTE: Generally there is a 1:1 mapping from vertex in voronoi diagram to delauney
    // triangle, but in the case of a degenerate vertex there may be more then one
    // triangle for a given vertex. We just ignore this case because it normally only
    // occurs if we created the Voronoi diagram from line segments with shared endpoints
    // (and we don't even give it segments!) see "is_degenerate" here:
    // https://www.boost.org/doc/libs/1_60_0/libs/polygon/doc/voronoi_diagram.htm
    // Code is a derivative of the answer to this:
    // https://stackoverflow.com/questions/34342038/how-to-triangulate-polygons-in-boost

    std::vector<Circle> empty_circles;
    for (auto vertex : diagram.vertices())
    {
        // We only want to consider vertices within our rectangle
        if (bounding_box.contains(Point(vertex.x(), vertex.y())))
        {
            std::vector<Point> triangle;
            auto edge = vertex.incident_edge();
            do
            {
                auto cell = edge->cell();
                if (!cell->contains_point())
                {
                    LOG(WARNING)
                        << "Found cell without point inside, something is likely seriously wrong";
                    continue;
                }

                triangle.push_back(points[cell->source_index()]);
                if (triangle.size() == 3)
                {
                    // Find the smallest distance from the vertex to a vertex on it's
                    // corresponding delauney triangle
                    std::vector<double> radii;
                    for (auto const &triangle_vertex : triangle)
                    {
                        radii.emplace_back(
                            (Point(vertex.x(), vertex.y()) - triangle_vertex).length());
                    }
                    double smallest_radius =
                        *std::min_element(radii.begin(), radii.end());

                    empty_circles.emplace_back(
                        Circle(Point(vertex.x(), vertex.y()), smallest_radius));

                    continue;
                }

                edge = edge->rot_next();
            } while (edge != vertex.incident_edge());
        }
    }
    return empty_circles;
}

#include "software/geom/algorithms/find_open_circles.h"

#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/furthest_point.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/algorithms/voronoi_diagram.h"

std::vector<Circle> findOpenCircles(const Rectangle &bounding_box,
                                    std::vector<Point> points)
{
    // We use a Voronoi Diagram and it's Delaunay triangulation to find the largest
    // open circles in the field
    // Reference: https://www.cs.swarthmore.edu/~adanner/cs97/s08/papers/schuster.pdf
    //
    // You can think of the Delauney triangulation as a way to connect all the points
    // (and the bounding_box corners) such that every point has three edges, and the
    // triangles formed are setup to be as regular as possible (ie. we avoid things
    // like super narrow triangles). The Voronoi diagram is the *dual* of this (scary
    // math words, I know), which just means you can construct it by taking the center
    // of each triangle and connecting it to the center of every adjacent triangle.
    //
    // So we can take each vertex on our voronoi diagram as the center of a open circle
    // on the field, and the size of the circle is the distance to the closest vertex
    // on the triangle that this vertex was created from

    // Filters out points that are outside of the bounding box
    points.erase(std::remove_if(points.begin(), points.end(),
                                [&bounding_box](const Point &p) {
                                    return !contains(bounding_box, p);
                                }),
                 points.end());

    std::vector<Circle> empty_circles;

    // Creating the Voronoi diagram with 2 or less points will produce no edges so we need
    // to handle these cases manually
    if (points.empty())
    {
        // If there are no points, return an empty vector since there are no constraints
        // to the size of the circle.
        return empty_circles;
    }
    if (points.size() == 1)
    {
        // If there is only 1 point, return circles centered at all four corners of the
        // bounding bounding_box.
        for (const Point &corner : bounding_box.getPoints())
        {
            empty_circles.emplace_back(
                Circle(corner, (points.front() - corner).length()));
        }
        return empty_circles;
    }
    if (points.size() == 2)
    {
        // If there are 2 point, split the points with a vector perpendicular to the
        // vector connecting the two points. Return 2 circles that are centered at the
        // points where the splitting vector intercepts the bounding_box. We should also
        // include circles centered at each of the corners.
        Vector connectedVec                     = points[1] - points[0];
        Point halfPoint                         = points[0] + (connectedVec * 0.5);
        Vector perpVec                          = connectedVec.perpendicular();
        std::unordered_set<Point> intersections = intersection(
            bounding_box,
            Segment(halfPoint +
                        (perpVec *
                         (furthestPoint(bounding_box, halfPoint) - halfPoint).length()),
                    halfPoint -
                        (perpVec *
                         (furthestPoint(bounding_box, halfPoint) - halfPoint).length())));
        std::vector<Point> corners = bounding_box.getPoints();
        std::copy(corners.begin(), corners.end(),
                  std::inserter(intersections, intersections.end()));
        for (const Point &intersect : intersections)
        {
            double radius =
                (findClosestPoint(intersect, points).value() - intersect).length();
            empty_circles.emplace_back(intersect, radius);
        }
        return empty_circles;
    }

    // Construct the voronoi diagram
    VoronoiDiagram vd(points);

    // The corners of the rectangles are locations for the centre of circles with their
    // radius being the distance to the corner's closest point.
    for (const Point &corner : bounding_box.getPoints())
    {
        Point closest = findClosestPoint(corner, points).value();
        empty_circles.emplace_back(Circle(corner, (corner - closest).length()));
    }

    std::vector<Point> intersects = vd.findVoronoiEdgeRecIntersects(bounding_box);

    // Radius of the circle will be the distance from the interception point
    // to the nearest input point.
    for (const Point &p : intersects)
    {
        double radius = (points[0] - p).length();
        for (const Point &inputP : points)
        {
            radius = std::min(radius, (inputP - p).length());
        }
        empty_circles.emplace_back(Circle(p, radius));
    }

    std::vector<Circle> calculatedEmptyCircles =
        vd.voronoiVerticesToOpenCircles(bounding_box);
    empty_circles.insert(empty_circles.end(), calculatedEmptyCircles.begin(),
                         calculatedEmptyCircles.end());

    // Sort the circles in descending order of radius
    std::sort(empty_circles.begin(), empty_circles.end(),
              [](auto c1, auto c2) { return c1.radius() > c2.radius(); });

    return empty_circles;
}

std::optional<Point> findClosestPoint(const Point &origin_point,
                                      const std::vector<Point> &test_points)
{
    std::optional<Point> closest_point = std::nullopt;

    if (!test_points.empty())
    {
        closest_point =
            *std::min_element(test_points.begin(), test_points.end(),
                              [&](const Point &test_point1, const Point &test_point2) {
                                  return (origin_point - test_point1).lengthSquared() <
                                         (origin_point - test_point2).lengthSquared();
                              });
    }

    return closest_point;
}

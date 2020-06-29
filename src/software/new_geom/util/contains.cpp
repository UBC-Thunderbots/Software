#include "software/new_geom/util/contains.h"

#include "software/new_geom/geom_constants.h"
#include "software/new_geom/util/almost_equal.h"
#include "software/new_geom/util/collinear.h"
#include "software/new_geom/util/distance.h"

bool contains(const Circle& container, const Segment& contained)
{
    return contains(container, contained.getSegStart()) &&
           contains(container, contained.getEnd());
}

bool contains(const Circle& container, const Point& contained)
{
    return distance(container.getOrigin(), contained) <= container.getRadius();
}

bool contains(const Polygon& container, const Point& contained)
{
    // This algorithm is from https://stackoverflow.com/a/16391873
    // but does not include the bounding boxes.
    //
    // A quick description of the algorithm (also from the same post) is as follows:
    // "I run a semi-infinite ray horizontally (increasing x, fixed y) out from the test
    // point, and count how many edges it crosses. At each crossing, the ray switches
    // between inside and outside. This is called the Jordan curve theorem."
    //
    // NOTE: This algorithm will treat boundaries on the bottom-left of the polygon
    // different from the boundaries on the top-right of the polygon. This small
    // inconsistency does not matter for our use cases, and actually has the benefit that
    // should two distinct polygons share an edge, any point along this edge will be
    // located in one and only one polygon.
    auto& points            = container.getPoints();
    bool point_is_contained = false;
    size_t i                = 0;
    size_t j                = points.size() - 1;
    double px               = contained.x();
    double py               = contained.y();
    while (i < points.size())
    {
        double pix                 = points[i].x();
        double piy                 = points[i].y();
        double pjx                 = points[j].x();
        double pjy                 = points[j].y();
        bool p_within_edge_y_range = (piy > py) != (pjy > py);
        bool p_in_half_plane_to_left_of_extended_edge =
            (px < (pjx - pix) * (py - piy) / (pjy - piy) + pix);

        if (p_within_edge_y_range && p_in_half_plane_to_left_of_extended_edge)
        {
            point_is_contained = !point_is_contained;
        }

        j = i++;
    }

    return point_is_contained;
}

bool contains(const Ray& container, const Point& contained)
{
    Point point_in_ray_direction = container.getStart() + container.toUnitVector();

    bool point_is_ray_start = contained == container.getStart();
    bool point_collinear_with_ray =
        collinear(contained, container.getStart(), point_in_ray_direction);
    bool point_is_in_ray_direction =
        ((contained - container.getStart()).normalize() - container.toUnitVector())
            .length() < FIXED_EPSILON;
    return point_is_ray_start || (point_collinear_with_ray && point_is_in_ray_direction);
}

bool contains(const Segment& container, const Point& contained, double fixed_epsilon,
              int ulps_distance)
{
    if (collinear(contained, container.getSegStart(), container.getEnd()))
    {
        // If the segment and contained are in a perfect vertical line, we must use Y
        // coordinate centric logic
        if (almostEqual(contained.x(), container.getEnd().x(), fixed_epsilon,
                        ulps_distance) &&
            almostEqual(container.getEnd().x(), container.getSegStart().x(),
                        fixed_epsilon, ulps_distance))
        {
            // Since segment and contained are collinear we only need to check one of the
            // coordinates, in this case we select Y because all X values are equal
            return (contained.y() <= container.getSegStart().y() &&
                    contained.y() >= container.getEnd().y()) ||
                   (contained.y() <= container.getEnd().y() &&
                    contained.y() >= container.getSegStart().y());
        }

        // Since segment and contained are collinear we only need to check one of the
        // coordinates, choose x because we know there is variance in these values
        return (contained.x() <= container.getSegStart().x() &&
                contained.x() >= container.getEnd().x()) ||
               (contained.x() <= container.getEnd().x() &&
                contained.x() >= container.getSegStart().x());
    }

    return false;
}

bool contains(const Rectangle& container, const Point& contained)
{
    auto& p = contained;
    auto& r = container;
    return p.x() >= r.negXNegYCorner().x() && p.y() >= r.negXNegYCorner().y() &&
           p.x() <= r.negXNegYCorner().x() + r.diagonal().x() &&
           p.y() <= r.negXNegYCorner().y() + r.diagonal().y();
}

#include "software/geom/polygon.h"

#include <unordered_set>

Polygon::Polygon(const std::vector<Point>& points)
    : points_(points), segments_(initSegments(points_))
{
    // we pre-compute the segments_ in the constructor to improve performance
}

Polygon::Polygon(const std::initializer_list<Point>& points)
    : Polygon(std::vector(points))
{
}

std::vector<Segment> Polygon::initSegments(std::vector<Point> points)
{
    std::vector<Segment> segments;
    for (unsigned i = 0; i < points.size(); i++)
    {
        // add a segment between consecutive points, but wrap index
        // to draw a segment from the last point to first point.
        segments.emplace_back(Segment{points[i], points[(i + 1) % points.size()]});
    }
    return segments;
}

Point Polygon::centroid() const
{
    // Explanation of the math/geometry behind this:
    // https://fotino.me/calculating-centroids/
    double x_centre    = 0;
    double y_centre    = 0;
    double signed_area = 0;

    for (unsigned i = 0; i < points_.size(); i++)
    {
        double x0 = points_[i].x();
        double y0 = points_[i].y();
        double x1 = points_[(i + 1) % points_.size()].x();
        double y1 = points_[(i + 1) % points_.size()].y();
        double a  = (x0 * y1) - (x1 * y0);

        x_centre += (x0 + x1) * a;
        y_centre += (y0 + y1) * a;
        signed_area += a;
    }

    return Point((Vector(x_centre, y_centre) / (3 * signed_area)));
}

Polygon Polygon::expand(double expansion_amount) const
{
    if (expansion_amount < 0)
    {
        throw std::invalid_argument(
            "Polygon::expand: expansion_amount must be non-negative");
    }
    Point centroid_point = centroid();
    std::vector<Point> expanded_points;
    expanded_points.reserve(points_.size());

    Vector last_expansion =
        (segments_[0].midPoint() - centroid_point).normalize(expansion_amount);
    Point first_point = segments_[0].getStart() + last_expansion;
    for (size_t i = 1; i < segments_.size(); i++)
    {
        Vector current_expansion =
            (segments_[i].midPoint() - centroid_point).normalize(expansion_amount);
        expanded_points.emplace_back(segments_[i].getStart() + current_expansion +
                                     last_expansion);
        last_expansion = current_expansion;
    }
    expanded_points.emplace_back(first_point + last_expansion);

    return Polygon(expanded_points);
}

Polygon Polygon::fromPoints(const Point& first_point, const Point& second_point)
{
    /*   The Polygon is constructed as follows:
     *
     *         ball_l           ball_r
     *           +-------+-------+
     *           |               |
     *           |    ball_c     |
     *           +-------+-------+
     *           |               |
     *           |               |
     *           |               |
     *           |     place_c   |
     *           +-------+-------+
     *           |               |
     *           |               |
     *           +-------+-------+
     *        place_l          place_r
     */

    const double RADIUS = 0.5;

    Vector first_to_second = first_point.toVector() - second_point.toVector();
    Vector second_to_first = -first_to_second;

    Point place_l = first_point + (first_to_second.normalize(RADIUS) +
                                   first_to_second.perpendicular().normalize(RADIUS));
    Point place_r = first_point + (first_to_second.normalize(RADIUS) -
                                   first_to_second.perpendicular().normalize(RADIUS));

    Point ball_l = second_point + (second_to_first.normalize(RADIUS) +
                                   second_to_first.perpendicular().normalize(RADIUS));
    Point ball_r = second_point + (second_to_first.normalize(RADIUS) -
                                   second_to_first.perpendicular().normalize(RADIUS));

    return Polygon({
        place_l,
        place_r,
        ball_l,
        ball_r,
    });
}

const std::vector<Segment>& Polygon::getSegments() const
{
    return segments_;
}

const std::vector<Point>& Polygon::getPoints() const
{
    return points_;
}

bool operator==(const Polygon& poly1, const Polygon& poly2)
{
    return (poly1.getPoints() == poly2.getPoints());
}

bool operator!=(const Polygon& poly1, const Polygon& poly2)
{
    return !(poly1 == poly2);
}

std::ostream& operator<<(std::ostream& os, const Polygon& poly)
{
    os << "Polygon with points {";
    for (const auto& pt : poly.getPoints())
    {
        os << pt << ' ';
    }
    os << '}';
    return os;
}

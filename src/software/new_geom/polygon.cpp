#include "software/new_geom/polygon.h"

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

bool Polygon::contains(const Point& p) const
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
    bool point_is_contained = false;
    unsigned i              = 0;
    unsigned j              = points_.size() - 1;
    double px               = p.x();
    double py               = p.y();
    while (i < points_.size())
    {
        double pix                 = points_[i].x();
        double piy                 = points_[i].y();
        double pjx                 = points_[j].x();
        double pjy                 = points_[j].y();
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

Polygon Polygon::expand(const Vector& expansion_vector) const
{
    // ASCII art showing an expanded Polygon
    //
    // Original Polygon:
    //
    //            B---------C
    //            |         |
    //            |         |
    //            A---------D
    //
    //                 |
    //                 V
    //          expansion vector
    //
    // Expanded Polygon (A and D are shifted down):
    //
    //            B---------C
    //            |         |
    //            |         |
    //            |         |
    //            A'--------D'
    //

    std::vector<Point> expanded_points;
    Point centroid_point = centroid();

    // left and right is with respect to the vector pointing straight up
    enum SideOfDividingLine
    {
        LEFT,
        RIGHT,
        ON_THE_LINE
    };

    // For a dividing line through centroid_point and perpendicular to the
    // expansion_vector, returns which side of the line the point p is on
    auto side_of_dividing_line = [&](const Point& p) {
        double d = (p - centroid_point).cross(expansion_vector.perpendicular());
        if (d == 0)
        {
            return SideOfDividingLine::ON_THE_LINE;
        }
        else if (d < 0)
        {
            return SideOfDividingLine::LEFT;
        }
        else
        {
            return SideOfDividingLine::RIGHT;
        }
    };

    SideOfDividingLine side_of_dividing_line_of_expansion_vector =
        side_of_dividing_line(centroid_point + expansion_vector);

    for (const auto& point : points_)
    {
        if (side_of_dividing_line(point) == side_of_dividing_line_of_expansion_vector)
        {
            expanded_points.push_back(point + expansion_vector);
        }
        else
        {
            expanded_points.push_back(point);
        }
    }
    return Polygon(expanded_points);
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

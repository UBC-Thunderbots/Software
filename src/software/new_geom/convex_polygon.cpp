#include "software/new_geom/convex_polygon.h"

#include <string>

ConvexPolygon::ConvexPolygon(const std::vector<Point>& points) : Polygon(points)
{
    if (!isConvex())
    {
        throw std::invalid_argument("Points do not make a convex polygon");
    }
}

ConvexPolygon::ConvexPolygon(const std::initializer_list<Point>& points) : Polygon(points)
{
    if (!isConvex())
    {
        throw std::invalid_argument("Points do not make a convex polygon");
    }
}

// From:
// https://math.stackexchange.com/questions/1743995/determine-whether-a-polygon-is-convex-based-on-its-vertices
bool ConvexPolygon::isConvex()
{
    if (points_.size() < 3)
    {
        return false;
    }

    double w_sign       = 0;
    double x_sign       = 0;
    double x_first_sign = 0;
    double x_flips      = 0;
    double y_sign       = 0;
    double y_first_sign = 0;
    double y_flips      = 0;

    Point prev = Point();
    Point curr = points_[points_.size() - 2];
    Point next = points_[points_.size() - 1];

    for (const Point& p : points_)
    {
        prev = curr;
        curr = next;
        next = p;

        Vector curr_to_prev = curr - prev;
        Vector next_to_curr = next - curr;

        // Calculate sign flips using the next edge vector ("next_to_curr"),
        // recording the first sign
        if (next_to_curr.x() > GeomConstants::FIXED_EPSILON)
        {
            if (x_sign == 0)
            {
                x_first_sign = 1;
            }
            else if (x_sign < -GeomConstants::FIXED_EPSILON)
            {
                x_flips = x_flips + 1;
            }
            x_sign = 1;
        }
        else if (next_to_curr.x() < -GeomConstants::FIXED_EPSILON)
        {
            if (x_sign == 0)
            {
                x_first_sign = -1;
            }
            else if (x_sign > GeomConstants::FIXED_EPSILON)
            {
                x_flips = x_flips + 1;
            }
            x_sign = -1;
        }

        if (x_flips > 2)
        {
            return false;
        }

        if (next_to_curr.y() > GeomConstants::FIXED_EPSILON)
        {
            if (y_sign == 0)
            {
                y_first_sign = 1;
            }
            else if (y_sign < -GeomConstants::FIXED_EPSILON)
            {
                y_flips = y_flips + 1;
            }
            y_sign = 1;
        }
        else if (next_to_curr.y() < -GeomConstants::FIXED_EPSILON)
        {
            if (y_sign == 0)
            {
                y_first_sign = -1;
            }
            else if (y_sign > GeomConstants::FIXED_EPSILON)
            {
                y_flips = y_flips + 1;
            }
            y_sign = -1;
        }

        if (y_flips > 2)
        {
            return false;
        }

        // Find out the orientation of this pair of edges and ensure it does not differ
        // from previous ones
        double w =
            curr_to_prev.x() * next_to_curr.y() - curr_to_prev.y() * next_to_curr.x();
        if (w_sign == 0 && w != 0)
        {
            w_sign = w;
        }
        else if (w_sign > GeomConstants::FIXED_EPSILON &&
                 w < -GeomConstants::FIXED_EPSILON)
        {
            return false;
        }
        else if (w_sign < -GeomConstants::FIXED_EPSILON &&
                 w > GeomConstants::FIXED_EPSILON)
        {
            return false;
        }
    }

    // Final sign flips
    if (x_sign != 0 && x_first_sign != 0 && x_sign != x_first_sign)
    {
        x_flips = x_flips + 1;
    }
    if (y_sign != 0 && y_first_sign != 0 && y_sign != y_first_sign)
    {
        y_flips = y_flips + 1;
    }

    // Concave polygons have two sign flips along each axis
    return (x_flips == 2 && y_flips == 2);
}

double ConvexPolygon::area() const
{
    // Algorithm taken from http://mathwords.com/a/area_convex_polygon.htm
    //
    // A = (1/2) * [(x1*y2 + x2y3 + x3y4 + ... + xny1) - (y1x2 + y2x3 + y3x4 + ... +
    // ynx1)] Coordinates must be taken in counterclockwise order around the polygon,
    // beginning and ending in the same point.
    std::vector<Point> reversePoints = points_;
    std::reverse(reversePoints.begin(), reversePoints.end());

    double first_term  = 0;
    double second_term = 0;

    unsigned num_points = reversePoints.size();

    for (unsigned i = 0; i < num_points; i++)
    {
        first_term += reversePoints[i].x() * reversePoints[(i + 1) % num_points].y();
        second_term += reversePoints[i].y() * reversePoints[(i + 1) % num_points].x();
    }

    return std::abs(first_term - second_term) / 2;
}

Point ConvexPolygon::centroid() const
{
    // Adapted from
    // https://www.geeksforgeeks.org/find-the-centroid-of-a-non-self-intersecting-closed-polygon/
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

ConvexPolygon ConvexPolygon::expand(const Vector& v) const
{
    // ASCII art showing an expanded ConvexPolygon
    //
    // Original ConvexPolygon:
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
    // Expanded ConvexPolygon (A and D are shifted down):
    //
    //            B---------C
    //            |         |
    //            |         |
    //            |         |
    //            A'--------D'
    //

    std::vector<Point> expanded_points;
    Point c = centroid();

    enum Orientation
    {
        LEFT,
        RIGHT,
        ON_THE_LINE
    };

    auto orientation = [&](const Point& p) {
        Point b1 = c + v.perpendicular();
        Point b2 = c - v.perpendicular();
        double d =
            (p.x() - b1.x()) * (b2.y() - b1.y()) - (p.y() - b1.y()) * (b2.x() - b1.x());
        if (d == 0)
        {
            return Orientation::ON_THE_LINE;
        }
        else if (d < 0)
        {
            return Orientation::LEFT;
        }
        else
        {
            return Orientation::RIGHT;
        }
    };

    Orientation orientation_of_v = orientation(c + v);

    for (const auto& p : points_)
    {
        if (orientation(p) == orientation_of_v)
        {
            expanded_points.push_back(p + v);
        }
        else
        {
            expanded_points.push_back(p);
        }
    }
    return ConvexPolygon(expanded_points);
}

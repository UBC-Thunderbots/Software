#include "software/geom/segment.h"

Segment::Segment(const Point& start, const Point& end) : start(start), end(end)
{
    if (start == end)
    {
        throw std::invalid_argument("Attempting to construct a degnerate segment");
    }
}

void Segment::setStart(const Point& o)
{
    start = o;
}

const Point& Segment::getStart() const
{
    return start;
}

void Segment::setEnd(const Point& f)
{
    end = f;
}

const Point& Segment::getEnd() const
{
    return end;
}

const Segment& Segment::reverse() const
{
    return Segment(end, start);
}

Vector Segment::toVector() const
{
    return end - start;
}

double Segment::length() const
{
    return (end - start).length();
}

bool Segment::operator==(const Segment& other) const
{
    return start == other.start && end == other.end;
}

bool Segment::contains(const Point& point, double fixed_epsilon, int ulps_distance) const
{
    if (Point::collinear(point, getStart(), getEnd()))
    {
        // If the segment and point are in a perfect vertical line, we must use Y
        // coordinate centric logic
        if (almostEqual(point.x(), getEnd().x(), fixed_epsilon, ulps_distance) &&
            almostEqual(getEnd().x(), getStart().x(), fixed_epsilon, ulps_distance))
        {
            // Since segment and point are collinear we only need to check one of the
            // coordinates, in this case we select Y because all X values are equal
            return (point.y() <= getStart().y() && point.y() >= getEnd().y()) ||
                   (point.y() <= getEnd().y() && point.y() >= getStart().y());
        }

        // Since segment and point are collinear we only need to check one of the
        // coordinates, choose x because we know there is variance in these values
        return (point.x() <= getStart().x() && point.x() >= getEnd().x()) ||
               (point.x() <= getEnd().x() && point.x() >= getStart().x());
    }

    return false;
}

bool Segment::collinear(const Segment& other, double fixed_epsilon,
                        int ulps_distance) const
{
    // Two segments are collinear if all Points are collinear
    if (Point::collinear(getStart(), getEnd(), other.getStart(), fixed_epsilon,
                         ulps_distance) &&
        Point::collinear(getStart(), getEnd(), other.getEnd(), fixed_epsilon,
                         ulps_distance))
    {
        return true;
    }
    return false;
}

bool Segment::intersects(const Segment& other) const
{
    // Using the FASTER LINE SEGMENT INTERSECTION algorithm from p.199 of Graphics Gems
    // III (IBM Version)
    // https://www.sciencedirect.com/science/article/pii/B9780080507552500452.

    // Values are pre-computed to improve performance
    const double p1x(getStart().x());
    const double p1y(getStart().y());
    const double p2x(getEnd().x());
    const double p2y(getEnd().y());
    const double p3x(other.getEnd().x());
    const double p3y(other.getEnd().y());
    const double p4x(other.getStart().x());
    const double p4y(other.getStart().y());

    const double ax = p2x - p1x;
    const double ay = p2y - p1y;
    const double bx = p3x - p4x;
    const double by = p3y - p4y;
    const double cx = p1x - p3x;
    const double cy = p1y - p3y;

    double denominator = ay * bx - ax * by;
    double numerator1  = by * cx - bx * cy;
    if (denominator > 0)
    {
        if (numerator1 < 0 || numerator1 > denominator)
        {
            return false;
        }
    }
    else
    {
        if (numerator1 > 0 || numerator1 < denominator)
        {
            return false;
        }
    }

    // Only compute numerator2 once we're sure we need it
    double numerator2 = ax * cy - ay * cx;
    if (denominator > 0)
    {
        if (numerator2 < 0 || numerator2 > denominator)
        {
            return false;
        }
    }
    else
    {
        if (numerator2 > 0 || numerator2 < denominator)
        {
            return false;
        }
    }

    return true;
}

std::vector<Point> Segment::intersection(const Segment& other) const
{
    std::vector<Point> output;

    Point a = getStart();
    Point b = getEnd();
    Point c = other.getStart();
    Point d = other.getEnd();

    // check for overlaps
    if (other.contains(a))
    {
        output.emplace_back(a);
    }
    if (other.contains(b))
    {
        output.emplace_back(b);
    }
    if (contains(c))
    {
        output.emplace_back(c);
    }
    if (contains(d))
    {
        output.emplace_back(d);
    }
    if (output.size() > 0)
    {
        return output;
    }

    auto intersection_value = Point::intersection(a, b, c, d);
    if (intersection_value)
    {
        if (contains(*intersection_value) && other.contains(*intersection_value))
        {
            output.emplace_back(*intersection_value);
        }
    }

    return output;
}

Point Segment::closestPointOnSeg(const Point& p) const
{
    Vector seg_vec       = toVector();
    double seg_vec_lensq = seg_vec.lengthSquared();

    // if one of the end-points is extremely close to the p point
    // then return 0.0
    if ((end - p).lengthSquared() < GeomConstants::FIXED_EPSILON)
    {
        return end;
    }

    if ((start - p).lengthSquared() < GeomConstants::FIXED_EPSILON)
    {
        return start;
    }

    // take care of 0 length segments
    if (seg_vec_lensq < GeomConstants::FIXED_EPSILON)
    {
        return start;
    }

    // find point C
    // which is the projection onto the line
    double lenseg = seg_vec.dot(p - start) / seg_vec.length();
    Point C       = start + lenseg * seg_vec.normalize();

    // check if C is in the line seg range
    double AC     = (start - C).lengthSquared();
    double BC     = (end - C).lengthSquared();
    double AB     = seg_vec_lensq;
    bool in_range = AC <= AB && BC <= AB;

    // if so return C
    if (in_range)
    {
        return C;
    }
    double lenA = (p - start).length();
    double lenB = (p - end).length();

    // otherwise return closest end of line-seg
    if (lenA < lenB)
    {
        return start;
    }
    return end;
}

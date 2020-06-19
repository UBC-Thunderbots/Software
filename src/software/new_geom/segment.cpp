#include "software/new_geom/segment.h"

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

void Segment::setEnd(const Point& o)
{
    end = o;
}

const Point& Segment::getEnd() const
{
    return end;
}

Segment Segment::reverse() const
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

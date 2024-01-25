#include "software/geom/algorithms/intersects.h"

#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersection.h"

bool intersects(const Polygon &first, const Segment &second)
{
    for (const auto &seg : first.getSegments())
    {
        if (intersects(seg, second))
        {
            return true;
        }
    }
    if (contains(first, second.getStart()))
    {
        return true;
    }
    return false;
}

bool intersects(const Segment &first, const Polygon &second)
{
    return intersects(second, first);
}

bool intersects(const Polygon &first, const Ray &second)
{
    for (const auto &seg : first.getSegments())
    {
        if (intersects(seg, second))
        {
            return true;
        }
    }
    return false;
}

bool intersects(const Ray &first, const Polygon &second)
{
    return intersects(second, first);
}

bool intersects(const Polygon &first, const Circle &second)
{
    if (contains(first, second.origin()))
    {
        return true;
    }
    for (const auto &seg : first.getSegments())
    {
        if (distance(seg, second.origin()) < second.radius())
        {
            return true;
        }
    }
    return false;
}

bool intersects(const Circle &first, const Polygon &second)
{
    return intersects(second, first);
}

bool intersects(const Circle &first, const Circle &second)
{
    return (first.origin() - second.origin()).lengthSquared() <
           std::pow(first.radius() + second.radius(), 2);
}

bool intersects(const Segment &first, const Circle &second)
{
    if (distance(first, second.origin()) <= second.radius())
    {
        return true;
    }

    return false;
}

bool intersects(const Circle &first, const Segment &second)
{
    return intersects(second, first);
}

bool intersects(const Segment &first, const Segment &second)
{
    // Using the FASTER LINE SEGMENT INTERSECTION algorithm from p.199 of Graphics Gems
    // III (IBM Version)
    // https://www.sciencedirect.com/science/article/pii/B9780080507552500452.

    // Values are pre-computed to improve performance
    const double p1x(first.getStart().x());
    const double p1y(first.getStart().y());
    const double p2x(first.getEnd().x());
    const double p2y(first.getEnd().y());
    const double p3x(second.getEnd().x());
    const double p3y(second.getEnd().y());
    const double p4x(second.getStart().x());
    const double p4y(second.getStart().y());

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

bool intersects(const Ray &first, const Segment &second)
{
    auto intersectionValue =
        intersection(first.getStart(), first.getStart() + first.toUnitVector(),
                     second.getStart(), second.getEnd());
    // If the infinitely long vectors defined by ray and segment intersect, check that the
    // intersection is within their definitions
    if (intersectionValue.has_value())
    {
        return contains(first, intersectionValue.value()) &&
               contains(second, intersectionValue.value());
    }
    // If there is no intersection, the ray and segment may be parallel, check if they are
    // overlapped
    return contains(first, second.getStart()) || contains(first, second.getEnd());
}

bool intersects(const Segment &first, const Ray &second)
{
    return intersects(second, first);
}

bool intersects(const Stadium &first, const Ray &second)
{
    auto start  = first.segment().getStart();
    auto end    = first.segment().getEnd();
    auto radius = first.radius();


    // The parametric definition of a ray is r(d)=o+tu where o is the origin point and u
    // is a unit vector Find the distance t along the ray that is closest to start and end
    // Distance cannot be negative because rays only go positively along the unit vector u
    auto startDist = std::fmax(0, second.toUnitVector().dot(start - second.getStart()));
    auto endDist   = std::fmax(0, second.toUnitVector().dot(end - second.getStart()));

    // Find corresponding point on the ray:
    auto startClosestPoint = second.getStart() + second.toUnitVector() * startDist;
    auto endClosestPoint   = second.getStart() + second.toUnitVector() * endDist;

    // Check if square of distance is less than square of radius
    // This will check if the ray is intersecting the circle with origin start
    // and circle with origin end, all that is left is to check the two line segments
    // connecting those circles
    auto startIntersecting = distanceSquared(start, startClosestPoint) <= radius * radius;
    auto endIntersecting   = distanceSquared(end, endClosestPoint) <= radius * radius;

    Vector normal =
        first.segment().toVector().rotate(Angle::fromDegrees(90)).normalize(radius);

    auto s1 = first.segment() + normal;
    auto s2 = first.segment() - normal;

    return startIntersecting || endIntersecting || intersects(second, s1) ||
           intersects(second, s2);
}

bool intersects(const Ray &first, const Stadium &second)
{
    return intersects(second, first);
}

bool intersects(const Stadium &first, const Circle &second)
{
    auto dist = distanceSquared(first.segment(), second.origin());

    return dist <= std::pow(first.radius() + second.radius(), 2);
}
bool intersects(const Circle &first, const Stadium &second)
{
    return intersects(second, first);
}

bool intersects(const Stadium &first, const Segment &second)
{
    auto startDistance    = distance(first.segment(), second.getStart());
    auto endDistance      = distance(first.segment(), second.getEnd());
    auto startDistanceSeg = distance(first.segment().getStart(), second);
    auto endDistanceSeg   = distance(first.segment().getEnd(), second);

    auto shortestDistance =
        std::fmin(startDistanceSeg,
                  std::fmin(endDistanceSeg, std::fmin(startDistance, endDistance)));


    return shortestDistance <= first.radius();
}
bool intersects(const Segment &first, const Stadium &second)
{
    return intersects(second, first);
}

bool intersects(const Stadium &first, const Polygon &second)
{
    for (const auto &seg : second.getSegments())
    {
        if (intersects(first, seg))
        {
            return true;
        }
    }
    return false;
}
bool intersects(const Polygon &first, const Stadium &second)
{
    return intersects(second, first);
}

bool intersects(const Stadium &first, const Stadium &second)
{
    auto startDistance    = distanceSquared(first.segment(), second.segment().getStart());
    auto endDistance      = distanceSquared(first.segment(), second.segment().getEnd());
    auto startDistanceSeg = distanceSquared(first.segment().getStart(), second.segment());
    auto endDistanceSeg   = distanceSquared(first.segment().getEnd(), second.segment());

    auto shortestDistanceSquared =
        std::fmin(startDistanceSeg,
                  std::fmin(endDistanceSeg, std::fmin(startDistance, endDistance)));


    return intersects(first.segment(), second.segment()) ||
           shortestDistanceSquared <= std::pow(first.radius() + second.radius(), 2);
}

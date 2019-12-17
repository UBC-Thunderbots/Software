#include "software/geom/util.h"

#include <algorithm>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/polygon/voronoi.hpp>
#include <cassert>
#include <cmath>
#include <g3log/g3log.hpp>
#include <iostream>
#include <limits>
#include <tuple>

#include "software/geom/rectangle.h"
#include "software/geom/segment.h"
#include "software/geom/voronoi_diagram.h"
#include "software/new_geom/angle.h"

double proj_length(const Segment &first, const Vector &second)
{
    return proj_length(first.toVector(), second - first.getSegStart().toVector());
}

double proj_length(const Vector &first, const Vector &second)
{
    return first.dot(second) / first.length();
}

double dist(const Point &first, const Point &second)
{
    return (first - second).length();
}

double dist(const Segment &first, const Segment &second)
{
    if (intersects(first, second))
    {
        return 0.0;
    }
    return std::sqrt(std::min(
        std::min(distsq(first, second.getSegStart()), distsq(first, second.getEnd())),
        std::min(distsq(second, first.getSegStart()), distsq(second, first.getEnd()))));
}

double dist(const Point &first, const Segment &second)
{
    return std::sqrt(distsq(first, second));
}

double dist(const Segment &first, const Point &second)
{
    return dist(second, first);
}

double dist(const Point &first, const Polygon &second)
{
    if (second.containsPoint(first))
    {
        return 0;
    }

    double min_dist = DBL_MAX;

    // Calculate the distance from the point to each edge
    for (auto &segment : second.getSegments())
    {
        double current_dist = dist(first, segment);
        if (current_dist < min_dist)
        {
            min_dist = current_dist;
        }
    }
    return min_dist;
}

double dist(const Point &first, const Rectangle &second)
{
    if (second.containsPoint(first))
    {
        return 0;
    }

    // Calculate the distance from the point to each edge of the rectangle
    std::array<double, 4> distances = {
        dist(first, Segment(second.posXPosYCorner(), second.posXNegYCorner())),
        dist(first, Segment(second.posXNegYCorner(), second.negXNegYCorner())),
        dist(first, Segment(second.negXNegYCorner(), second.negXPosYCorner())),
        dist(first, Segment(second.negXPosYCorner(), second.posXPosYCorner()))};
    return *std::min_element(distances.begin(), distances.end());
}

double distsq(const Point &first, const Segment &second)
{
    double seglensq    = lengthSquared(second);
    Vector relsecond_s = first - second.getSegStart();
    Vector relsecond_e = first - second.getEnd();

    Vector s_vec2 = second.toVector();

    if (s_vec2.dot(relsecond_s) > 0 && second.reverse().toVector().dot(relsecond_e) > 0)
    {
        if (isDegenerate(second))
        {
            return relsecond_s.length();
        }
        double cross = relsecond_s.cross(s_vec2);
        return std::fabs(cross * cross / seglensq);
    }

    double lensq_s = distsq(second.getSegStart(), first),
           lensq_e = distsq(second.getEnd(), first);

    return std::min(lensq_s, lensq_e);
}

double distsq(const Segment &first, const Point &second)
{
    return distsq(second, first);
}

double distsq(const Point &first, const Point &second)
{
    return (first - second).lengthSquared();
}

bool isDegenerate(const Segment &segment)
{
    return distsq(segment.getSegStart(), segment.getEnd()) < EPS2;
}

double length(const Segment &segment)
{
    return dist(segment.getSegStart(), segment.getEnd());
}

double lengthSquared(const Segment &segment)
{
    return distsq(segment.getSegStart(), segment.getEnd());
}

bool contains(const LegacyTriangle &out, const Point &in)
{
    double angle = 0;
    for (int i = 0, j = 2; i < 3; j = i++)
    {
        if ((in - out[i]).length() < EPS)
        {
            return true;  // SPECIAL CASE
        }
        double a =
            atan2((out[i] - in).cross(out[j] - in), (out[i] - in).dot(out[j] - in));
        angle += a;
    }
    return std::fabs(angle) > 6;
}

bool contains(const Circle &out, const Point &in)
{
    return distsq(out.getOrigin(), in) <= out.getRadius() * out.getRadius();
}

bool contains(const Circle &out, const Segment &in)
{
    return dist(in, out.getOrigin()) < out.getRadius();
}

bool contains(const Segment &out, const Point &in)
{
    if (collinear(in, out.getSegStart(), out.getEnd()))
    {
        // If the segment and point are in a perfect vertical line, we must use Y
        // coordinate centric logic
        if ((std::abs(in.x() - out.getEnd().x()) < EPS) &&
            (std::abs(out.getEnd().x() - out.getSegStart().x()) < EPS))
        {
            // if collinear we only need to check one of the coordinates,
            // in this case we select Y because all X values are equal
            return (in.y() <= out.getSegStart().y() && in.y() >= out.getEnd().y()) ||
                   (in.y() <= out.getEnd().y() && in.y() >= out.getSegStart().y());
        }

        // if collinear we only need to check one of the coordinates,
        // choose x because we know there is variance in these values
        return (in.x() <= out.getSegStart().x() && in.x() >= out.getEnd().x()) ||
               (in.x() <= out.getEnd().x() && in.x() >= out.getSegStart().x());
    }

    return false;
}

bool contains(const Ray &out, const Point &in)
{
    Point point_in_ray_direction = out.getStart() + out.toUnitVector();
    if (collinear(in, out.getStart(), point_in_ray_direction) &&
        (((in - out.getStart()).normalize() - out.toUnitVector()).length() < EPS))
    {
        return true;
    }
    return false;
}

bool contains(const Rectangle &out, const Point &in)
{
    return out.containsPoint(in);
}

bool intersects(const LegacyTriangle &first, const Circle &second)
{
    return contains(first, second.getOrigin()) ||
           dist(getSide(first, 0), second.getOrigin()) < second.getRadius() ||
           dist(getSide(first, 1), second.getOrigin()) < second.getRadius() ||
           dist(getSide(first, 2), second.getOrigin()) < second.getRadius();
}
bool intersects(const Circle &first, const LegacyTriangle &second)
{
    return intersects(second, first);
}

bool intersects(const Circle &first, const Circle &second)
{
    return (first.getOrigin() - second.getOrigin()).length() <
           (first.getRadius() + second.getRadius());
}

bool intersects(const Ray &first, const Segment &second)
{
    auto isect =
        lineIntersection(first.getStart(), first.getStart() + first.toUnitVector(),
                         second.getSegStart(), second.getEnd());
    // If the infinitely long vectors defined by ray and segment intersect, check that the
    // intersection is within their definitions
    if (isect.has_value())
    {
        return contains(first, isect.value()) && contains(second, isect.value());
    }
    // If there is no intersection, the ray and segment may be parallel, check if they are
    // overlapped
    return contains(second, first.getStart());
}
bool intersects(const Segment &first, const Ray &second)
{
    return intersects(second, first);
}

bool intersects(const Segment &first, const Circle &second)
{
    // if the segment is inside the circle AND at least one of the points is
    // outside the circle
    return contains(second, first) && (distsq(first.getSegStart(), second.getOrigin()) >
                                           second.getRadius() * second.getRadius() ||
                                       distsq(first.getEnd(), second.getOrigin()) >
                                           second.getRadius() * second.getRadius());
}
bool intersects(const Circle &first, const Segment &second)
{
    return intersects(second, first);
}

bool intersects(const Segment &first, const Segment &second)
{
    boost::geometry::model::segment<Point> AB(first.getSegStart(), first.getEnd());
    boost::geometry::model::segment<Point> CD(second.getSegStart(),
                                              second.getEnd());  // similar code

    return boost::geometry::intersects(AB, CD);
}

template <size_t N>
Point getVertex(const LegacyPolygon<N> &poly, unsigned int i)
{
    if (i > N)
        throw std::out_of_range("poly does not have that many sides!!!");
    else
        return poly[i];
}

template <size_t N>
void setVertex(LegacyPolygon<N> &poly, unsigned int i, const Vector &v)
{
    if (i > N)
        throw std::out_of_range("poly does not have that many sides!!!");
    else
        poly[i] = v;
}

template <size_t N>
Segment getSide(const LegacyPolygon<N> &poly, unsigned int i)
{
    return Segment(getVertex(poly, i), getVertex(poly, (i + 1) % N));
}

std::vector<Shot> angleSweepCirclesAll(const Point &src, const Point &p1, const Point &p2,
                                       const std::vector<Point> &obstacles,
                                       const double &radius)
{
    Angle p1_angle = (p1 - src).orientation();
    Angle p2_angle = (p2 - src).orientation();

    Angle start_angle = std::min(p1_angle, p2_angle);
    Angle end_angle   = std::max(p1_angle, p2_angle);

    // This handles the special case where the start and end angle straddle the
    // negative y axis, which causes some issues with angles "ticking over" from pi to
    // -pi and vice-versa
    if (end_angle - start_angle > Angle::half())
    {
        Angle start_angle_new = start_angle + (end_angle - start_angle).angleMod();
        end_angle             = start_angle;
        start_angle           = start_angle_new;
    }

    if (collinear(src, p1, p2))
    {
        // return a result that contains the direction of the line and zero angle if not
        // blocked by obstacles
        Segment collinear_seg = Segment(src, p1);
        for (Point p : obstacles)
        {
            if (intersects(collinear_seg, Circle(p, radius)))
            {
                // intersection with obstacle found, we're done here and we return nothing
                return {};
            }
        }

        return {Shot(Point(collinear_seg.toVector()), Angle::zero())};
    }

    // "Sweep" a line from the `src` to the target line segment, and create an "event"
    // whenever the line enters or leaves an obstacle, int value of `-1` to indicate the
    // sweep "leaving" an obstacle, and `+1` to indicate the sweep "entering" another
    // obstacle
    // The angle for each event is measured relative to the start angle
    std::vector<std::pair<Angle, int>> events;
    for (const Point &obstacle : obstacles)
    {
        Vector diff = obstacle - src;
        if (diff.length() < radius)
        {
            // `src` is within `radius` of this obstacle
            return {};
        }

        const Angle cent   = (diff.orientation() - start_angle).angleMod();
        const Angle span   = Angle::asin(radius / diff.length());
        const Angle range1 = cent - span;
        const Angle range2 = cent + span;

        if (range1 < Angle::zero() && range2 > end_angle - start_angle)
        {
            // Obstacle takes up entire angle we are sweeping
            return {};
        }

        if (range1 < -Angle::half() || range2 > Angle::half())
        {
            continue;
        }
        if (range1 > Angle::zero() && range1 < end_angle - start_angle)
        {
            events.push_back(std::make_pair(range1, -1));
        }
        if (range2 > Angle::zero() && range2 < end_angle - start_angle)
        {
            events.push_back(std::make_pair(range2, 1));
        }
    }

    if (events.empty())
    {
        // No obstacles in the way, so just return a range hitting the entire target
        // line segment
        return {
            Shot(Point((p1.toVector() + p2.toVector()) / 2), end_angle - start_angle)};
    }

    // Sort the events by angle
    std::sort(events.begin(), events.end());

    // Collapse all contiguous sections of "+1" and "-1" respectively, as these represent
    // overlapping obstacles (from the perspective of the `src` point to the target line
    // segment)
    std::vector<std::pair<Angle, int>> events_collapsed;
    for (auto &event : events)
    {
        if (events_collapsed.empty() || event.second != events_collapsed.back().second)
        {
            events_collapsed.emplace_back(event);
        }
    }

    if (events_collapsed[0].second == -1)
    {
        events_collapsed.insert(events_collapsed.begin(),
                                std::make_pair(Angle::zero(), 1));
    }
    if (events_collapsed.back().second == 1)
    {
        events_collapsed.emplace_back(std::make_pair(end_angle - start_angle, -1));
    }

    std::vector<Shot> result;
    for (unsigned i = 1; i < events_collapsed.size(); i += 2)
    {
        // Calculate the center of this range on the target line segement
        Angle range_start = events_collapsed[i - 1].first + start_angle;
        Angle range_end   = events_collapsed[i].first + start_angle;
        Angle mid         = (range_end - range_start) / 2 + range_start;
        Vector ray        = Vector::createFromAngle(mid) * 10.0;
        Point inter       = lineIntersection(src, src + ray, p1, p2).value();

        // Offset the final values by the start angle
        result.emplace_back(Shot(inter, range_end - range_start));
    }

    return result;
}

std::optional<Shot> angleSweepCircles(const Point &src, const Point &p1, const Point &p2,
                                      const std::vector<Point> &obstacles,
                                      const double &radius)
{
    // Get all possible shots we could take
    std::vector<Shot> possible_shots =
        angleSweepCirclesAll(src, p1, p2, obstacles, radius);

    // Sort by the interval angle (ie. the open angle the shot is going through)
    std::sort(possible_shots.begin(), possible_shots.end(),
              [](auto s1, auto s2) { return s1.getOpenAngle() > s2.getOpenAngle(); });

    // Return the shot through the largest open interval if there are any
    if (possible_shots.empty())
    {
        return std::nullopt;
    }
    return possible_shots[0];
}

std::vector<Point> circleBoundaries(const Point &centre, double radius, int num_points)
{
    Angle rotate_amount = Angle::full() / num_points;
    std::vector<Point> ans;
    Vector bound(radius, 0.0);
    for (int i = 0; i < num_points; i++)
    {
        Point temp = centre + bound;
        ans.push_back(temp);
        bound = bound.rotate(rotate_amount);
    }
    return ans;
}

bool collinear(const Point &a, const Point &b, const Point &c)
{
    if ((a - b).lengthSquared() < EPS2 || (b - c).lengthSquared() < EPS2 ||
        (a - c).lengthSquared() < EPS2)
    {
        return true;
    }
    return std::fabs((b - a).cross(c - a)) < EPS;
}

Point clipPoint(const Point &p, const Point &bound1, const Point &bound2)
{
    const double minx = std::min(bound1.x(), bound2.x());
    const double miny = std::min(bound1.y(), bound2.y());
    const double maxx = std::max(bound1.x(), bound2.x());
    const double maxy = std::max(bound1.y(), bound2.y());
    Point ret         = p;
    if (p.x() < minx)
    {
        ret.set(minx, ret.y());
    }
    else if (p.x() > maxx)
    {
        ret.set(maxx, ret.y());
    }
    if (p.y() < miny)
    {
        ret.set(ret.x(), miny);
    }
    else if (p.y() > maxy)
    {
        ret.set(ret.x(), maxy);
    }
    return ret;
}

Point clipPoint(const Point &p, const Rectangle &r)
{
    const double minx = r.negXNegYCorner().x();
    const double miny = r.negXNegYCorner().y();
    const double maxx = r.posXPosYCorner().x();
    const double maxy = r.posXPosYCorner().y();
    Point ret         = p;
    if (p.x() < minx)
    {
        ret.set(minx, ret.y());
    }
    else if (p.x() > maxx)
    {
        ret.set(maxx, ret.y());
    }
    if (p.y() < miny)
    {
        ret.set(ret.x(), miny);
    }
    else if (p.y() > maxy)
    {
        ret.set(ret.x(), maxy);
    }
    return ret;
}

std::vector<Point> lineCircleIntersect(const Point &centre, double radius,
                                       const Point &segA, const Point &segB)
{
    std::vector<Point> ans;

    // take care of 0 length segments too much error here
    if ((segB - segA).lengthSquared() < EPS)
    {
        return ans;
    }

    double lenseg = (segB - segA).dot(centre - segA) / (segB - segA).length();
    Point C       = segA + lenseg * (segB - segA).normalize();

    // if C outside circle no intersections
    if ((C - centre).lengthSquared() > radius * radius + EPS)
    {
        return ans;
    }

    // if C on circle perimeter return the only intersection
    if ((C - centre).lengthSquared() < radius * radius + EPS &&
        (C - centre).lengthSquared() > radius * radius - EPS)
    {
        ans.push_back(C);
        return ans;
    }
    // first possible intersection
    double lensegb = radius * radius - (C - centre).lengthSquared();

    ans.push_back(C - (lensegb * (segB - segA).normalize()));
    ans.push_back(C + lensegb * (segB - segA).normalize());

    return ans;
}

std::vector<Point> lineRectIntersect(const Rectangle &r, const Point &segA,
                                     const Point &segB)
{
    std::vector<Point> ans;
    for (unsigned int i = 0; i < 4; i++)
    {
        const Point &a = r[i];
        // to draw a line segment from point 3 to point 0
        const Point &b = r[(i + 1) % 4];
        if (intersects(Segment(a, b), Segment(segA, segB)) &&
            uniqueLineIntersects(a, b, segA, segB))
        {
            ans.push_back(lineIntersection(a, b, segA, segB).value());
        }
    }
    return ans;
}

Point vectorRectIntersect(const Rectangle &r, const Point &pointA, const Point &pointB)
{
    std::vector<Point> points =
        lineRectIntersect(r, pointA, pointA + ((pointB - pointA) * 100));
    for (Point i : points)
    {
        if (contains(Ray(pointA, (pointB - pointA)), i))
        {
            return i;
        }
    }
    return Point(1.0 / 0.0, 1.0 / 0.0);  // no solution found, propagate infinity
}


Point closestPointOnSeg(const Point &p, const Segment &segment)
{
    return closestPointOnSeg(p, segment.getSegStart(), segment.getEnd());
}
Point closestPointOnSeg(const Point &centre, const Point &segA, const Point &segB)
{
    // if one of the end-points is extremely close to the centre point
    // then return 0.0
    if ((segB - centre).lengthSquared() < EPS2)
    {
        return segB;
    }

    if ((segA - centre).lengthSquared() < EPS2)
    {
        return segA;
    }

    // take care of 0 length segments
    if ((segB - segA).lengthSquared() < EPS2)
    {
        return segA;
    }

    // find point C
    // which is the projection onto the line
    double lenseg = (segB - segA).dot(centre - segA) / (segB - segA).length();
    Point C       = segA + lenseg * (segB - segA).normalize();

    // check if C is in the line seg range
    double AC     = (segA - C).lengthSquared();
    double BC     = (segB - C).lengthSquared();
    double AB     = (segA - segB).lengthSquared();
    bool in_range = AC <= AB && BC <= AB;

    // if so return C
    if (in_range)
    {
        return C;
    }
    double lenA = (centre - segA).length();
    double lenB = (centre - segB).length();

    // otherwise return closest end of line-seg
    if (lenA < lenB)
    {
        return segA;
    }
    return segB;
}

bool uniqueLineIntersects(const Point &a, const Point &b, const Point &c, const Point &d)
{
    return std::abs((d - c).cross(b - a)) > EPS;
}

std::vector<Point> lineIntersection(const Segment &a, const Segment &b)
{
    if (std::fabs((b.getEnd() - b.getSegStart()).cross(a.getEnd() - a.getSegStart())) <
        EPS)
    {
        // parallel line segments, find if they're collinear and return the 2 points
        // on the line they both lay on if they are collinear and intersecting
        // shamelessly copypasted from
        // https://stackoverflow.com/questions/22456517/algorithm-for-finding-the-segment-overlapping-two-collinear-segments
        if (collinear(a.getSegStart(), b.getSegStart(), b.getEnd()) &&
            collinear(a.getEnd(), b.getSegStart(), b.getEnd()))
        {
            double slope = (a.getEnd().y() - a.getSegStart().y()) /
                           (a.getEnd().x() - a.getSegStart().x());
            bool isHorizontal = slope < EPS;
            bool isDescending = slope < 0 && !isHorizontal;
            double invertY    = isDescending || isHorizontal ? -1 : 1;

            Point min1 =
                Point(std::min(a.getSegStart().x(), a.getEnd().x()),
                      std::min(a.getSegStart().y() * invertY, a.getEnd().y() * invertY));
            Point max1 =
                Point(std::max(a.getSegStart().x(), a.getEnd().x()),
                      std::max(a.getSegStart().y() * invertY, a.getEnd().y() * invertY));

            Point min2 =
                Point(std::min(b.getSegStart().x(), b.getEnd().x()),
                      std::min(b.getSegStart().y() * invertY, b.getEnd().y() * invertY));
            Point max2 =
                Point(std::max(b.getSegStart().x(), b.getEnd().x()),
                      std::max(b.getSegStart().y() * invertY, b.getEnd().y() * invertY));

            Point minIntersection;
            if (isDescending)
                minIntersection = Point(std::max(min1.x(), min2.x()),
                                        std::min(min1.y() * invertY, min2.y() * invertY));
            else
                minIntersection = Point(std::max(min1.x(), min2.x()),
                                        std::max(min1.y() * invertY, min2.y() * invertY));

            Point maxIntersection;
            if (isDescending)
                maxIntersection = Point(std::min(max1.x(), max2.x()),
                                        std::max(max1.y() * invertY, max2.y() * invertY));
            else
                maxIntersection = Point(std::min(max1.x(), max2.x()),
                                        std::min(max1.y() * invertY, max2.y() * invertY));

            bool intersect =
                minIntersection.x() <= maxIntersection.x() &&
                ((!isDescending && minIntersection.y() <= maxIntersection.y()) ||
                 (isDescending && minIntersection.y() >= maxIntersection.y()));

            if (intersect)
            {
                return std::vector<Point>{minIntersection, maxIntersection};
            }
            else
                return std::vector<Point>();
        }
        else
            return std::vector<Point>();
    }

    return std::vector<Point>{
        a.getSegStart() +
        (a.getSegStart() - b.getSegStart()).cross(b.getEnd() - b.getSegStart()) /
            (b.getEnd() - b.getSegStart()).cross(a.getEnd() - a.getSegStart()) *
            (a.getEnd() - a.getSegStart())};
}

// shamelessly copy-pasted from RoboJackets
std::optional<Point> lineIntersection(const Point &a, const Point &b, const Point &c,
                                      const Point &d)
{
    Segment line1(a, b), line2(c, d);
    double x1 = line1.getSegStart().x();
    double y1 = line1.getSegStart().y();
    double x2 = line1.getEnd().x();
    double y2 = line1.getEnd().y();
    double x3 = line2.getSegStart().x();
    double y3 = line2.getSegStart().y();
    double x4 = line2.getEnd().x();
    double y4 = line2.getEnd().y();

    double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (denom == 0)
    {
        // log the parallel lines when we actually implement logging?
        return std::nullopt;
    }

    double deta = x1 * y2 - y1 * x2;
    double detb = x3 * y4 - y3 * x4;

    Point intersection;

    intersection.set((deta * (x3 - x4) - (x1 - x2) * detb) / denom,
                     (deta * (y3 - y4) - (y1 - y2) * detb) / denom);

    return std::make_optional(intersection);
}

std::pair<std::optional<Point>, std::optional<Point>> raySegmentIntersection(
    const Ray &ray, const Segment &segment)
{
    Point ray2 = ray.getStart() + ray.toUnitVector();

    std::optional<Point> intersection =
        lineIntersection(ray.getStart(), ray2, segment.getSegStart(), segment.getEnd());

    // If there exists a single intersection, and it exists on the ray and within the
    // segment
    if (intersection.has_value() && contains(ray, intersection.value()) &&
        contains(segment, intersection.value()))
    {
        return std::make_pair(intersection, std::nullopt);
    }
    // The ray and segment are parallel, and collinear
    else if (!intersection.has_value() &&
             collinear(ray.getStart(), segment.getSegStart(), segment.getEnd()))
    {
        // Check if ray passes through both segment start and end
        if (ray.toUnitVector() == (segment.getSegStart() - ray.getStart()).normalize() &&
            ray.toUnitVector() == (segment.getEnd() - ray.getStart()).normalize())
        {
            return std::make_pair(segment.getSegStart(), segment.getEnd());
        }

        // Since we know the ray and segment are overlapping (with ray origin within the
        // segment), return the ray start position, and the end of the segment that is in
        // the direction of the ray
        ray.toUnitVector() == (segment.getEnd() - segment.getSegStart()).normalize()
            ? intersection = std::make_optional(segment.getEnd())
            : intersection = std::make_optional(segment.getSegStart());
        return std::make_pair(ray.getStart(), intersection.value());
    }
    // The ray and segment do not intersect at all
    else
    {
        return std::make_pair(std::nullopt, std::nullopt);
    }
}

std::pair<std::optional<Point>, std::optional<Point>> rayRectangleIntersection(
    const Ray &ray, const Rectangle &rectangle)
{
    std::vector<Segment> rectangle_segments = {
        Segment(rectangle.posXPosYCorner(), rectangle.negXPosYCorner()),
        Segment(rectangle.negXPosYCorner(), rectangle.negXNegYCorner()),
        Segment(rectangle.negXNegYCorner(), rectangle.posXNegYCorner()),
        Segment(rectangle.posXNegYCorner(), rectangle.posXPosYCorner()),
    };
    std::pair<std::optional<Point>, std::optional<Point>> result =
        std::make_pair(std::nullopt, std::nullopt);
    for (const auto &seg : rectangle_segments)
    {
        auto intersection = raySegmentIntersection(ray, seg);
        // Always take the result with more non-nullopt values
        if ((intersection.first && !result.first) ||
            (intersection.second && !result.second))
        {
            result = intersection;
        }
    }

    return result;
}

std::optional<Point> getRayIntersection(Ray ray1, Ray ray2)
{
    // Calculate if the intersecion exists along segments of infinite length
    std::optional<Point> intersection =
        lineIntersection(ray1.getStart(), ray1.getStart() + ray1.toUnitVector(),
                         ray2.getStart(), ray2.getStart() + ray2.toUnitVector());

    // Return if no intersection exists
    if (!intersection.has_value())
    {
        return std::nullopt;
    }

    // Check of the intersection exits along the direction of both rays
    if (((intersection.value() - ray1.getStart()).normalize() == ray1.toUnitVector()) &&
        (intersection.value() - ray2.getStart()).normalize() == ray2.toUnitVector())
    {
        return intersection.value();
    }
    else
    {
        return std::nullopt;
    }
}

Vector reflect(const Vector &v, const Vector &n)
{
    if (n.length() < EPS)
    {
        return v;
    }
    Vector normal = n.normalize();
    return v - 2 * v.dot(normal) * normal;
}

Point reflect(const Point &a, const Point &b, const Point &p)
{
    // Make a as origin.
    // Rotate by 90 degrees, does not matter which direction?
    Vector n = (b - a).rotate(Angle::quarter());
    return a + reflect(p - a, n);
}

Point calcBlockCone(const Vector &a, const Vector &b, const double &radius)
{
    if (a.length() < EPS || b.length() < EPS)
    {
    }
    // unit vector and bisector
    Vector au = a / a.length();
    Vector c  = au + b / b.length();
    // use similar triangle
    return Point(c * (radius / std::fabs(au.cross(c))));
}

Point calcBlockCone(const Point &a, const Point &b, const Point &p, const double &radius)
{
    return p + (calcBlockCone(a - p, b - p, radius)).toVector();
}

Vector calcBlockOtherRay(const Point &a, const Point &c, const Point &g)
{
    return reflect(c - a, g - c);  // this, and the next two instances, were
                                   // changed from a - c since reflect() was
                                   // fixed
}

double offsetToLine(Point x0, Point x1, Point p)
{
    Vector n;

    // get normal to line
    n = (x1 - x0).perpendicular().normalize();

    return fabs(n.dot(p - x0));
}

double offsetAlongLine(Point x0, Point x1, Point p)
{
    Vector n, v;

    // get normal to line
    n = x1 - x0;
    n = n.normalize();

    v = p - x0;

    return n.dot(v);
}

Point segmentNearLine(Point a0, Point a1, Point b0, Point b1)
{
    Vector v, n;
    Point p;
    double dn, t;

    v = a1 - a0;
    n = (b1 - b0).normalize();
    n = n.perpendicular();

    dn = v.dot(n);
    if (std::fabs(dn) < EPS)
    {
        return a0;
    }

    t = -(a0 - b0).dot(n) / dn;

    if (t < 0)
    {
        t = 0;
    }
    if (t > 1)
    {
        t = 1;
    }
    p = a0 + v * t;

    return p;
}

Point intersection(Point a1, Point a2, Point b1, Point b2)
{
    Vector a = a2 - a1;

    Vector b1r = (b1 - a1).rotate(-a.orientation());
    Vector b2r = (b2 - a1).rotate(-a.orientation());
    Vector br  = (b1r - b2r);

    return Vector(b2r.x() - b2r.y() * (br.x() / br.y()), 0.0).rotate(a.orientation()) +
           a1;
}

Angle acuteVertexAngle(Vector v1, Vector v2)
{
    return v1.orientation().minDiff(v2.orientation());
}

Angle acuteVertexAngle(Point p1, Point p2, Point p3)
{
    return acuteVertexAngle(p1 - p2, p3 - p2);
}

double closestPointTime(Point x1, Vector v1, Point x2, Vector v2)
{
    Vector v  = v1 - v2;
    double sl = v.lengthSquared();
    double t;

    if (sl < EPS)
    {
        return 0.0;  // parallel tracks, any time is ok.
    }
    t = -v.dot(x1 - x2) / sl;
    if (t < 0.0)
    {
        return 0.0;  // nearest time was in the past, now is closest point from
                     // now on.
    }
    return t;
}

bool pointInFrontVector(Point offset, Vector direction, Point p)
{
    // compare angle different
    Angle a1   = direction.orientation();
    Angle a2   = (p - offset).orientation();
    Angle diff = (a1 - a2).angleMod();
    return diff < Angle::quarter() && diff > -Angle::quarter();
}

std::pair<Point, Point> getCircleTangentPoints(const Point &start, const Circle &circle,
                                               double buffer)
{
    // If the point is already inside the circe arccos won't work so just return
    // the perp points
    if (contains(circle, start))
    {
        double perpDist = std::sqrt(circle.getRadius() * circle.getRadius() -
                                    (circle.getOrigin() - start).lengthSquared());
        Point p1 =
            start +
            (circle.getOrigin() - start).perpendicular().normalize(perpDist + buffer);
        Point p2 =
            start -
            ((circle.getOrigin() - start).perpendicular().normalize(perpDist + buffer));
        return std::make_pair(p1, p2);
    }
    else
    {
        double radiusAngle =
            std::acos(circle.getRadius() / (start - circle.getOrigin()).length());
        Point p1 = circle.getOrigin() + (start - circle.getOrigin())
                                            .rotate(Angle::fromRadians(radiusAngle))
                                            .normalize(circle.getRadius() + buffer);
        Point p2 = circle.getOrigin() + (start - circle.getOrigin())
                                            .rotate(-Angle::fromRadians(radiusAngle))
                                            .normalize(circle.getRadius() + buffer);
        return std::make_pair(p1, p2);
    }
}

std::pair<Ray, Ray> getCircleTangentRays(const Point reference, const Circle circle,
                                         double buffer)
{
    auto [tangent_point1, tangent_point2] =
        getCircleTangentPoints(reference, circle, buffer);

    return std::make_pair(Ray(tangent_point1, (tangent_point1 - reference).normalize()),
                          Ray(tangent_point2, (tangent_point2 - reference).normalize()));
}

bool pointIsRightOfLine(const Segment &line, const Point &point)
{
    return (line.getEnd().x() - line.getSegStart().x()) *
                   (point.y() - line.getSegStart().y()) -
               (line.getEnd().y() - line.getSegStart().y()) *
                   (point.x() - line.getSegStart().x()) <
           0.0;
}

Point getPointsMean(const std::vector<Point> &points)
{
    Point average = Point(0, 0);
    for (unsigned int i = 0; i < points.size(); i++)
    {
        average += points[i].toVector();
    }

    Vector averageVector = average.toVector();

    averageVector /= static_cast<double>(points.size());
    return Point(averageVector);
}

double getPointsVariance(const std::vector<Point> &points)
{
    Point mean = getPointsMean(points);

    double sum = 0.0;
    for (unsigned int i = 0; i < points.size(); i++)
    {
        sum += (points[i] - mean).lengthSquared();
    }

    sum /= static_cast<double>(points.size());
    return sqrt(sum);
}

std::optional<Segment> segmentEnclosedBetweenRays(Segment segment, Ray ray1, Ray ray2)
{
    // Create rays located at the extremes of the segment, that point in the direction
    // outwards are parallel to the segment
    const Ray extremes1 =
        Ray(segment.getEnd(), Vector(segment.getEnd() - segment.getSegStart()));
    const Ray extremes2 =
        Ray(segment.getSegStart(), Vector(segment.getSegStart() - segment.getEnd()));

    const std::optional<Point> extreme_intersect11 = getRayIntersection(extremes1, ray1);
    const std::optional<Point> extreme_intersect12 = getRayIntersection(extremes2, ray1);
    const std::optional<Point> extreme_intersect21 = getRayIntersection(extremes1, ray2);
    const std::optional<Point> extreme_intersect22 = getRayIntersection(extremes2, ray2);

    // Check for the cases that the rays intersect the same segment projection
    if ((extreme_intersect11.has_value() == extreme_intersect21.has_value()) ||
        (extreme_intersect21.has_value() == extreme_intersect22.has_value()))
    {
        return std::nullopt;
    }
    else
    {
        // Since we know that both rays aren't passing through the same side of the
        // segment at this point, then as long as they both only intersect 1 point the
        // segment must be enclosed between them
        if ((extreme_intersect11.has_value() != extreme_intersect12.has_value()) &&
            (extreme_intersect21.has_value() != extreme_intersect22.has_value()))
        {
            return std::make_optional(segment);
        }
        // Covers the case where a single ray passes by both sides of the segment
        else
        {
            return std::nullopt;
        }
    }
}

std::optional<Segment> getIntersectingSegment(Ray ray1, Ray ray2, Segment segment)
{
    // Check if the segment is enclosed between the rays
    if (segmentEnclosedBetweenRays(segment, ray1, ray2))
    {
        return segment;
    }

    // Calculate intersections of each individual ray and the segment
    auto [intersect11, intersect12] = raySegmentIntersection(ray1, segment);
    auto [intersect21, intersect22] = raySegmentIntersection(ray2, segment);

    // Check if there are any real intersections
    if (!intersect11.has_value() && !intersect21.has_value())
    {
        return std::nullopt;
    }
    // Check if one of the rays is overlapping the segment. If this is the case, return
    // the segment (If a ray intersects a ray more than one time it must be overlapping)
    else if ((intersect11.has_value() && intersect12.has_value()) ||
             (intersect21.has_value() && intersect22.has_value()))
    {
        return segment;
    }
    // If there is only one intersection point for each ray combine the intersections into
    // a segment
    else if ((intersect11.has_value() && !intersect12.has_value()) &&
             (intersect21.has_value() && !intersect22.has_value()))
    {
        return std::make_optional(Segment(intersect11.value(), intersect21.value()));
    }
    // If only one ray intersects the segment return the segment between the intersection
    // and the segment extreme (intersection11 is real, intersection22 is not)
    else if (intersect11.has_value() && !intersect21.has_value())
    {
        const Ray extremes1 =
            Ray(segment.getEnd(), Vector(segment.getEnd() - segment.getSegStart()));
        const Ray extremes2 =
            Ray(segment.getSegStart(), Vector(segment.getSegStart() - segment.getEnd()));
        ;

        std::optional<Point> extreme_intersect1 = getRayIntersection(extremes1, ray2);
        std::optional<Point> extreme_intersect2 = getRayIntersection(extremes2, ray2);

        if (extreme_intersect1.has_value())
        {
            return std::make_optional(Segment(intersect11.value(), segment.getEnd()));
        }
        else if (extreme_intersect2.has_value())
        {
            return std::make_optional(
                Segment(intersect11.value(), segment.getSegStart()));
        }
    }
    // If only one ray intersects the segment return the segment between the intersection
    // and the segment extreme (intersection11 is real, intersection22 is not)
    else if (intersect11.has_value() && !intersect21.has_value())
    {
        const Ray extremes1 =
            Ray(segment.getEnd(), Vector(segment.getEnd() - segment.getSegStart()));
        const Ray extremes2 =
            Ray(segment.getSegStart(), Vector(segment.getSegStart() - segment.getEnd()));
        ;

        std::optional<Point> extreme_intersect1 = getRayIntersection(extremes1, ray1);
        std::optional<Point> extreme_intersect2 = getRayIntersection(extremes2, ray1);

        if (extreme_intersect1.has_value())
        {
            return std::make_optional(Segment(intersect21.value(), segment.getEnd()));
        }
        else if (extreme_intersect2.has_value())
        {
            return std::make_optional(
                Segment(intersect21.value(), segment.getSegStart()));
        }
    }
    // All cases have been checked, return std::nullopt
    return std::nullopt;
}

std::optional<Segment> mergeOverlappingParallelSegments(Segment segment1,
                                                        Segment segment2)
{
    std::optional<Segment> redundant_segment =
        mergeFullyOverlappingSegments(segment1, segment2);

    // If the segments are not parallel, then return std::nullopt. (The segments are
    // parallel of all points are collinear)
    if (!collinear(segment1.getSegStart(), segment1.getEnd(), segment2.getSegStart()) &&
        !collinear(segment1.getSegStart(), segment1.getEnd(), segment2.getEnd()))
    {
        return std::nullopt;
    }
    // Check the case where one segment is completely contained in the other
    else if (redundant_segment.has_value())
    {
        return redundant_segment;
    }
    // Check if the beginning of segment2 lays inside segment1
    else if (contains(segment1, segment2.getSegStart()))
    {
        // If segment2.getSegStart() lays in segment1, then the combined segment is
        // segment2,getEnd() and the point furthest from segmen2.getEnd()
        return (segment1.getSegStart() - segment2.getEnd()).lengthSquared() >
                       (segment1.getEnd() - segment2.getEnd()).lengthSquared()
                   ? Segment(segment1.getSegStart(), segment2.getEnd())
                   : Segment(segment1.getEnd(), segment2.getEnd());
    }
    // Now check if the end of segment2 lays inside segment1
    else if (contains(segment1, segment2.getEnd()))
    {
        // If segment2.getSegStart() lays in segment1, then the combined segment is
        // segment2,getEnd() and the point furtherst from segmen2.getEnd()
        return (segment1.getSegStart() - segment2.getSegStart()).lengthSquared() >
                       (segment1.getEnd() - segment2.getSegStart()).lengthSquared()
                   ? Segment(segment1.getSegStart(), segment2.getSegStart())
                   : Segment(segment1.getEnd(), segment2.getSegStart());
    }
    return std::nullopt;
}

std::optional<Segment> mergeFullyOverlappingSegments(Segment segment1, Segment segment2)
{
    // If the segments are not parallel, then return std::nullopt. (The segments are
    // parallel if all points are collinear)
    if (!collinear(segment1.getSegStart(), segment1.getEnd(), segment2.getSegStart()) &&
        !collinear(segment1.getSegStart(), segment1.getEnd(), segment2.getEnd()))
    {
        return std::nullopt;
    }

    Segment largest_segment, smallest_segment;
    // Grab the largest segment
    if (segment1.toVector().lengthSquared() > segment2.toVector().lengthSquared())
    {
        largest_segment  = segment1;
        smallest_segment = segment2;
    }
    else
    {
        largest_segment  = segment2;
        smallest_segment = segment1;
    }

    // The segment is redundant if both points of the smallest segment are contained in
    // the largest segment
    if (contains(largest_segment, smallest_segment.getSegStart()) &&
        contains(largest_segment, smallest_segment.getEnd()))
    {
        return std::make_optional(largest_segment);
    }
    else
    {
        return std::nullopt;
    }
}

int calcBinaryTrespassScore(const Rectangle &rectangle, const Point &point)
{
    if (rectangle.containsPoint(point))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


std::vector<Circle> findOpenCircles(Rectangle bounding_box, std::vector<Point> points)
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
                                    return !bounding_box.containsPoint(p);
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
        for (Point &corner : bounding_box.corners())
        {
            empty_circles.emplace_back(Circle(corner, dist(points.front(), corner)));
        }
        return empty_circles;
    }
    if (points.size() == 2)
    {
        // If there are 2 point, split the points with a vector perpendicular to the
        // vector connecting the two points. Return 2 circles that are centered at the
        // points where the splitting vector intercepts the bounding_box. We should also
        // include circles centered at each of the corners.
        Vector connectedVec           = points[1] - points[0];
        Point halfPoint               = points[0] + (connectedVec * 0.5);
        Vector perpVec                = connectedVec.perpendicular();
        std::vector<Point> intersects = lineRectIntersect(
            bounding_box,
            halfPoint +
                (perpVec * dist(bounding_box.furthestCorner(halfPoint), halfPoint)),
            halfPoint -
                (perpVec * dist(bounding_box.furthestCorner(halfPoint), halfPoint)));
        std::vector<Point> corners = bounding_box.corners();
        intersects.insert(intersects.end(), corners.begin(), corners.end());
        for (const Point &intersect : intersects)
        {
            double radius = dist(findClosestPoint(intersect, points).value(), intersect);
            empty_circles.emplace_back(intersect, radius);
        }
        return empty_circles;
    }

    // Construct the voronoi diagram
    VoronoiDiagram vd(points);

    // The corners of the rectangles are locations for the centre of circles with their
    // radius being the distance to the corner's closest point.
    for (const Point &corner : bounding_box.corners())
    {
        Point closest = findClosestPoint(corner, points).value();
        empty_circles.emplace_back(Circle(corner, dist(corner, closest)));
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
              [](auto c1, auto c2) { return c1.getRadius() > c2.getRadius(); });

    return empty_circles;
}

Polygon circleToPolygon(const Circle &circle, size_t num_points)
{
    std::vector<Point> points;
    for (unsigned i = 0; i < num_points; i++)
    {
        Point p = circle.getOrigin() +
                  Vector(circle.getRadius(), 0)
                      .rotate(Angle::fromDegrees((360.0 / num_points) * i));
        points.emplace_back(p);
    }
    return Polygon(points);
}

std::optional<Point> findClosestPoint(const Point &origin_point,
                                      std::vector<Point> test_points)
{
    std::optional<Point> closest_point = std::nullopt;

    if (!test_points.empty())
    {
        closest_point = *std::min_element(
            test_points.begin(), test_points.end(),
            [&](const Point &test_point1, const Point &test_point2) {
                return dist(origin_point, test_point1) < dist(origin_point, test_point2);
            });
    }

    return closest_point;
}

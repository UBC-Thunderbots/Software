#include "geom/util.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <tuple>

#include "geom/angle.h"
#include "geom/rectangle.h"
#include "geom/segment.h"


double proj_len(const Segment &first, const Vector &second)
{
    return proj_len(first.toVector(), second - first.getSegStart());
}

double proj_len(const Vector &first, const Vector &second)
{
    return first.dot(second) / first.len();
}

double dist(const Vector &first, const Vector &second)
{
    return (first - second).len();
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

double dist(const Line &first, const Vector &second)
{
    if (isDegenerate(first))
    {
        return dist(first.getFirst(), second);
    }
    return fabs((second - first.getFirst()).cross(first.getSecond() - first.getFirst()) /
                (first.getSecond() - first.getFirst()).len());
}

double dist(const Vector &first, const Line &second)
{
    return dist(second, first);
}

double dist(const Vector &first, const Segment &second)
{
    return std::sqrt(distsq(first, second));
}

double dist(const Segment &first, const Vector &second)
{
    return dist(second, first);
}

double dist(const Point &first, const Rectangle &second)
{
    if (second.containsPoint(first))
    {
        return 0;
    }

    // Calculate the distance from the point to each edge of the rectangle
    std::array<double, 4> distances = {
        dist(first, Segment(second.neCorner(), second.seCorner())),
        dist(first, Segment(second.seCorner(), second.swCorner())),
        dist(first, Segment(second.swCorner(), second.nwCorner())),
        dist(first, Segment(second.nwCorner(), second.neCorner()))};
    return *std::min_element(distances.begin(), distances.end());
}

double distsq(const Vector &first, const Segment &second)
{
    double seglensq    = lensq(second);
    Vector relsecond_s = first - second.getSegStart();
    Vector relsecond_e = first - second.getEnd();

    Vector s_vec2 = second.toVector();

    if (s_vec2.dot(relsecond_s) > 0 && second.reverse().toVector().dot(relsecond_e) > 0)
    {
        if (isDegenerate(second))
        {
            return relsecond_s.len();
        }
        double cross = relsecond_s.cross(s_vec2);
        return std::fabs(cross * cross / seglensq);
    }

    double lensq_s = distsq(second.getSegStart(), first),
           lensq_e = distsq(second.getEnd(), first);

    return std::min(lensq_s, lensq_e);
}

double distsq(const Segment &first, const Vector &second)
{
    return distsq(second, first);
}

double distsq(const Vector &first, const Vector &second)
{
    return (first - second).lensq();
}

bool isDegenerate(const Segment &segment)
{
    return distsq(segment.getSegStart(), segment.getEnd()) < EPS2;
}

bool isDegenerate(const Line &line)
{
    return distsq(line.getFirst(), line.getSecond()) < EPS2;
}

bool isDegenerate(const Ray &ray)
{
    return distsq(ray.getRayStart(), ray.getDirection()) < EPS2;
}

double len(const Segment &segment)
{
    return dist(segment.getSegStart(), segment.getEnd());
}

double lensq(const Segment &segment)
{
    return distsq(segment.getSegStart(), segment.getEnd());
}

double lensq(const Line &line)
{
    (void)line;  // unused
    return std::numeric_limits<double>::infinity();
}

bool contains(const LegacyTriangle &out, const Point &in)
{
    double angle = 0;
    for (int i = 0, j = 2; i < 3; j = i++)
    {
        if ((in - out[i]).len() < EPS)
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
        if ((in.x() - out.getEnd().x() == 0) &&
            (out.getEnd().x() - out.getSegStart().x() == 0))
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
    Point point_in_ray_direction = out.getRayStart() + out.getDirection();
    if (collinear(in, out.getRayStart(), point_in_ray_direction) &&
        (in - out.getRayStart()).norm() == out.getDirection().norm())
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
    return (first.getOrigin() - second.getOrigin()).len() <
           (first.getRadius() + second.getRadius());
}

bool intersects(const Ray &first, const Segment &second)
{
    auto isect =
        lineIntersection(first.getRayStart(), first.getRayStart() + first.getDirection(),
                         second.getSegStart(), second.getEnd());
    // If the infinitely long vectors defined by ray and segment intersect, check that the
    // intersection is within their definitions
    if (isect.has_value())
    {
        return contains(first, isect.value()) && contains(second, isect.value());
    }
    // If there is no intersection, the ray and segment may be parallel, check if they are
    // overlapped
    return contains(second, first.getRayStart());
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
    if (sign((first.getSegStart() - first.getEnd())
                 .cross(second.getSegStart() - second.getEnd())) == 0)
    {
        // find distance of two endpoints on segments furthest away from each
        // other
        double mx_len = std::sqrt(
            std::max(std::max((second.getSegStart() - first.getEnd()).lensq(),
                              (second.getEnd() - first.getEnd()).lensq()),
                     std::max((second.getSegStart() - first.getSegStart()).lensq(),
                              (second.getEnd() - first.getSegStart()).lensq())));
        // if the segments cross then this distance should be less than
        // the sum of the distances of the line segments
        return mx_len < (first.getSegStart() - first.getEnd()).len() +
                            (second.getSegStart() - second.getEnd()).len() + EPS;
    }

    return sign((first.getEnd() - first.getSegStart())
                    .cross(second.getSegStart() - first.getSegStart())) *
                   sign((first.getEnd() - first.getSegStart())
                            .cross(second.getEnd() - first.getSegStart())) <=
               0 &&
           sign((second.getEnd() - second.getSegStart())
                    .cross(first.getSegStart() - second.getSegStart())) *
                   sign((second.getEnd() - second.getSegStart())
                            .cross(first.getEnd() - second.getSegStart())) <=
               0;
}

template <size_t N>
Vector getVertex(const LegacyPolygon<N> &poly, unsigned int i)
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

std::vector<std::pair<Point, Angle>> angleSweepCirclesAll(
    const Vector &src, const Vector &p1, const Vector &p2,
    const std::vector<Point> &obstacles, const double &radius)
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
        return {std::make_pair(collinear_seg.toVector(), Angle::zero())};
    }

    // "Sweep" a line from the `src` to the target line segment, and create an "event"
    // whenever the line enters or leaves an obstacle, int value of `-1` to indicate the
    // sweep "leaving" an obstacle, and `+1` to indicate the sweep "entering" another
    // obstacle
    // The angle for each event is measured relative to the start angle
    std::vector<std::pair<Angle, int>> events;
    for (const Vector &obstacle : obstacles)
    {
        Vector diff = obstacle - src;
        if (diff.len() < radius)
        {
            // `src` is within `radius` of this obstacle
            return {};
        }

        const Angle cent   = (diff.orientation() - start_angle).angleMod();
        const Angle span   = Angle::asin(radius / diff.len());
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
        return {std::make_pair((p1 + p2) / 2, end_angle - start_angle)};
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
    std::vector<std::pair<Point, Angle>> result;
    for (int i = 0; i < events_collapsed.size() - 1; i += 2)
    {
        // Calculate the center of this range on the target line segement
        Angle range_start = events_collapsed[i].first + start_angle;
        Angle range_end   = events_collapsed[i + 1].first + start_angle;
        Angle mid         = (range_end - range_start) / 2 + range_start;
        Vector ray        = Vector::createFromAngle(mid) * 10.0;
        Vector inter      = lineIntersection(src, src + ray, p1, p2).value();

        // Offset the final values by the start angle
        result.emplace_back(std::make_pair(inter, range_end - range_start));
    }

    return result;
}

std::optional<std::pair<Point, Angle>> angleSweepCircles(
    const Vector &src, const Vector &p1, const Vector &p2,
    const std::vector<Vector> &obstacles, const double &radius)
{
    // Get all possible shots we could take
    std::vector<std::pair<Point, Angle>> possible_shots =
        angleSweepCirclesAll(src, p1, p2, obstacles, radius);

    // Sort by the interval angle (ie. the open angle the shot is going through)
    std::sort(possible_shots.begin(), possible_shots.end(),
              [](auto p1, auto p2) { return p1.second > p2.second; });

    // Return the shot through the largest open interval if there are any
    if (possible_shots.empty())
    {
        return std::nullopt;
    }
    return possible_shots[0];
}

std::vector<Vector> circleBoundaries(const Vector &centre, double radius, int num_points)
{
    Angle rotate_amount = Angle::full() / num_points;
    std::vector<Vector> ans;
    Vector bound(radius, 0.0);
    for (int i = 0; i < num_points; i++)
    {
        Vector temp = centre + bound;
        ans.push_back(temp);
        bound = bound.rotate(rotate_amount);
    }
    return ans;
}

bool collinear(const Vector &a, const Vector &b, const Vector &c)
{
    if ((a - b).lensq() < EPS2 || (b - c).lensq() < EPS2 || (a - c).lensq() < EPS2)
    {
        return true;
    }
    return std::fabs((b - a).cross(c - a)) < EPS;
}

Vector clipPoint(const Vector &p, const Vector &bound1, const Vector &bound2)
{
    const double minx = std::min(bound1.x(), bound2.x());
    const double miny = std::min(bound1.y(), bound2.y());
    const double maxx = std::max(bound1.x(), bound2.x());
    const double maxy = std::max(bound1.y(), bound2.y());
    Vector ret        = p;
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

Vector clipPoint(const Vector &p, const Rectangle &r)
{
    const double minx = r.swCorner().x();
    const double miny = r.swCorner().y();
    const double maxx = r.neCorner().x();
    const double maxy = r.neCorner().y();
    Vector ret        = p;
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

std::vector<Vector> lineCircleIntersect(const Vector &centre, double radius,
                                        const Vector &segA, const Vector &segB)
{
    std::vector<Vector> ans;

    // take care of 0 length segments too much error here
    if ((segB - segA).lensq() < EPS)
    {
        return ans;
    }

    double lenseg = (segB - segA).dot(centre - segA) / (segB - segA).len();
    Vector C      = segA + lenseg * (segB - segA).norm();

    // if C outside circle no intersections
    if ((C - centre).lensq() > radius * radius + EPS)
    {
        return ans;
    }

    // if C on circle perimeter return the only intersection
    if ((C - centre).lensq() < radius * radius + EPS &&
        (C - centre).lensq() > radius * radius - EPS)
    {
        ans.push_back(C);
        return ans;
    }
    // first possible intersection
    double lensegb = radius * radius - (C - centre).lensq();

    ans.push_back(C - lensegb * (segB - segA).norm());
    ans.push_back(C + lensegb * (segB - segA).norm());

    return ans;
}

std::vector<Vector> lineRectIntersect(const Rectangle &r, const Vector &segA,
                                      const Vector &segB)
{
    std::vector<Vector> ans;
    for (unsigned int i = 0; i < 4; i++)
    {
        const Vector &a = r[i];
        // to draw a line segment from point 3 to point 0
        const Vector &b = r[(i + 1) % 4];
        if (intersects(Segment(a, b), Segment(segA, segB)) &&
            uniqueLineIntersects(a, b, segA, segB))
        {
            ans.push_back(lineIntersection(a, b, segA, segB).value());
        }
    }
    return ans;
}

Vector vectorRectIntersect(const Rectangle &r, const Vector &vecA, const Vector &vecB)
{
    std::vector<Vector> points = lineRectIntersect(r, vecA, (vecB - vecA) * 100 + vecA);
    for (Vector i : points)
    {
        if (contains(Ray(vecA, (vecB - vecA)), i))
        {
            return i;
        }
    }
    return Vector(1.0 / 0.0, 1.0 / 0.0);  // no solution found, propagate infinity
}

Vector closestPointOnSeg(const Vector &centre, const Vector &segA, const Vector &segB)
{
    // if one of the end-points is extremely close to the centre point
    // then return 0.0
    if ((segB - centre).lensq() < EPS2)
    {
        return segB;
    }

    if ((segA - centre).lensq() < EPS2)
    {
        return segA;
    }

    // take care of 0 length segments
    if ((segB - segA).lensq() < EPS2)
    {
        return segA;
    }

    // find point C
    // which is the projection onto the line
    double lenseg = (segB - segA).dot(centre - segA) / (segB - segA).len();
    Vector C      = segA + lenseg * (segB - segA).norm();

    // check if C is in the line seg range
    double AC     = (segA - C).lensq();
    double BC     = (segB - C).lensq();
    double AB     = (segA - segB).lensq();
    bool in_range = AC <= AB && BC <= AB;

    // if so return C
    if (in_range)
    {
        return C;
    }
    double lenA = (centre - segA).len();
    double lenB = (centre - segB).len();

    // otherwise return closest end of line-seg
    if (lenA < lenB)
    {
        return segA;
    }
    return segB;
}

Vector closestPointOnLine(const Vector &centre, const Vector &lineA, const Vector &lineB)
{
    // find point C, the projection onto the line
    double len_line = (lineB - lineA).dot(centre - lineA) / (lineB - lineA).len();
    Vector C        = lineA + len_line * (lineB - lineA).norm();
    return C;

    // check if C is in the line range
    double AC     = (lineA - C).lensq();
    double BC     = (lineB - C).lensq();
    double AB     = (lineA - lineB).lensq();
    bool in_range = AC <= AB && BC <= AB;

    // if so return C
    if (in_range)
    {
    }

    double lenA = (centre - lineA).len();
    double lenB = (centre - lineB).len();

    // otherwise return closest end of line-seg
    if (lenA < lenB)
    {
        return lineA;
    }
    return lineB;
}

namespace
{
    std::vector<Vector> lineseg_circle_intersect(const Vector &centre, double radius,
                                                 const Vector &segA, const Vector &segB)
    {
        std::vector<Vector> ans;
        std::vector<Vector> poss = lineCircleIntersect(centre, radius, segA, segB);

        for (Vector i : poss)
        {
            bool x_ok = i.x() <= std::max(segA.x(), segB.x()) + EPS &&
                        i.x() >= std::min(segA.x(), segB.x()) - EPS;
            bool y_ok = i.y() <= std::max(segA.y(), segB.y()) + EPS &&
                        i.y() >= std::min(segA.y(), segB.y()) - EPS;
            if (x_ok && y_ok)
            {
                ans.push_back(i);
            }
        }
        return ans;
    }
}  // namespace

bool uniqueLineIntersects(const Vector &a, const Vector &b, const Vector &c,
                          const Vector &d)
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
std::optional<Point> lineIntersection(const Vector &a, const Vector &b, const Vector &c,
                                      const Vector &d)
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
    Ray &ray, Segment &segment)
{
    Point ray2 = ray.getRayStart() + ray.getDirection();

    std::optional<Point> intersection = lineIntersection(
        ray.getRayStart(), ray2, segment.getSegStart(), segment.getEnd());

    // If there exists a single intersection, and it exists on the ray and within the
    // segment
    if (intersection.has_value() && contains(ray, intersection.value()) &&
        contains(segment, intersection.value()))
    {
        return std::make_pair(intersection, std::nullopt);
    }
    // The ray and segment are parallel, and collinear
    else if (!intersection.has_value() &&
             collinear(ray.getRayStart(), segment.getSegStart(), segment.getEnd()))
    {
        // Check if ray passes through both segment start and end
        if (ray.getDirection().norm() ==
                (segment.getSegStart() - ray.getRayStart()).norm() &&
            ray.getDirection().norm() == (segment.getEnd() - ray.getRayStart()).norm())
        {
            return std::make_pair(segment.getSegStart(), segment.getEnd());
        }

        // Since we know the ray and segment are overlapping (with ray origin within the
        // segment), return the ray start position, and the end of the segment that is in
        // the direction of the ray
        ray.getDirection().norm() == (segment.getEnd() - segment.getSegStart()).norm()
            ? intersection = std::make_optional(segment.getEnd())
            : intersection = std::make_optional(segment.getSegStart());
        return std::make_pair(ray.getRayStart(), intersection.value());
    }
    // The ray and segment do not intersect at all
    else
    {
        return std::make_pair(std::nullopt, std::nullopt);
    }
}

std::optional<Point> getRayIntersection(Ray ray1, Ray ray2)
{
    // Calculate if the intersecion exists along segments of infinite length
    std::optional<Point> intersection =
        lineIntersection(ray1.getRayStart(), ray1.getRayStart() + ray1.getDirection(),
                         ray2.getRayStart(), ray2.getRayStart() + ray2.getDirection());

    // Return if no intersection exists
    if (!intersection.has_value())
    {
        return std::nullopt;
    }

    // Check of the intersection exits along the direction of both rays
    if (((intersection.value() - ray1.getRayStart()).norm() ==
         ray1.getDirection().norm()) &&
        (intersection.value() - ray2.getRayStart()).norm() == ray2.getDirection().norm())
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
    if (n.len() < EPS)
    {
        return v;
    }
    Vector normal = n.norm();
    return v - 2 * v.dot(normal) * normal;
}

Vector reflect(const Vector &a, const Vector &b, const Vector &p)
{
    // Make a as origin.
    // Rotate by 90 degrees, does not matter which direction?
    Vector n = (b - a).rotate(Angle::quarter());
    return a + reflect(p - a, n);
}

Vector calcBlockCone(const Vector &a, const Vector &b, const double &radius)
{
    if (a.len() < EPS || b.len() < EPS)
    {
    }
    // unit vector and bisector
    Vector au = a / a.len();
    Vector c  = au + b / b.len();
    // use similar triangle
    return c * (radius / std::fabs(au.cross(c)));
}

Vector calcBlockCone(const Vector &a, const Vector &b, const Vector &p,
                     const double &radius)
{
    return p + calcBlockCone(a - p, b - p, radius);
}

Vector calcBlockOtherRay(const Vector &a, const Vector &c, const Vector &g)
{
    return reflect(c - a, g - c);  // this, and the next two instances, were
                                   // changed from a - c since reflect() was
                                   // fixed
}

Vector calcBlockConeDefender(const Vector &a, const Vector &b, const Vector &c,
                             const Vector &g, const double &r)
{
    Vector R = reflect(c - a, g - c);
    return calcBlockCone(R + c, b, c, r);
}

double offsetToLine(Vector x0, Vector x1, Vector p)
{
    Vector n;

    // get normal to line
    n = (x1 - x0).perp().norm();

    return fabs(n.dot(p - x0));
}

double offsetAlongLine(Vector x0, Vector x1, Vector p)
{
    Vector n, v;

    // get normal to line
    n = x1 - x0;
    n = n.norm();

    v = p - x0;

    return n.dot(v);
}

Vector segmentNearLine(Vector a0, Vector a1, Vector b0, Vector b1)
{
    Vector v, n, p;
    double dn, t;

    v = a1 - a0;
    n = (b1 - b0).norm();
    n = n.perp();

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

Vector intersection(Vector a1, Vector a2, Vector b1, Vector b2)
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
    double sl = v.lensq();
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

bool pointInFrontVector(Vector offset, Vector direction, Vector p)
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
                                    (circle.getOrigin() - start).lensq());
        Point p1 = start + (circle.getOrigin() - start).perp().norm(perpDist + buffer);
        Point p2 = start - (circle.getOrigin() - start).perp().norm(perpDist + buffer);
        return std::make_pair(p1, p2);
    }
    else
    {
        double radiusAngle =
            std::acos(circle.getRadius() / (start - circle.getOrigin()).len());
        Point p1 = circle.getOrigin() + (start - circle.getOrigin())
                                            .rotate(Angle::ofRadians(radiusAngle))
                                            .norm(circle.getRadius() + buffer);
        Point p2 = circle.getOrigin() + (start - circle.getOrigin())
                                            .rotate(-Angle::ofRadians(radiusAngle))
                                            .norm(circle.getRadius() + buffer);
        return std::make_pair(p1, p2);
    }
}

std::pair<Ray, Ray> getCircleTangentRays(const Point reference, const Circle circle,
                                         double buffer)
{
    auto [tangent_point1, tangent_point2] =
        getCircleTangentPoints(reference, circle, buffer);

    return std::make_pair(Ray(tangent_point1, (tangent_point1 - reference).norm()),
                          Ray(tangent_point2, (tangent_point2 - reference).norm()));
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
        average += points[i];
    }

    average /= static_cast<double>(points.size());
    return average;
}

double getPointsVariance(const std::vector<Point> &points)
{
    Point mean = getPointsMean(points);

    double sum = 0.0;
    for (unsigned int i = 0; i < points.size(); i++)
    {
        sum += (points[i] - mean).lensq();
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
    else
    {
        return std::nullopt;
    }
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
        return (segment1.getSegStart() - segment2.getEnd()).lensq() >
                       (segment1.getEnd() - segment2.getEnd()).lensq()
                   ? Segment(segment1.getSegStart(), segment2.getEnd())
                   : Segment(segment1.getEnd(), segment2.getEnd());
    }
    // Now check if the end of segment2 lays inside segment1
    else if (contains(segment1, segment2.getEnd()))
    {
        // If segment2.getSegStart() lays in segment1, then the combined segment is
        // segment2,getEnd() and the point furtherst from segmen2.getEnd()
        return (segment1.getSegStart() - segment2.getSegStart()).lensq() >
                       (segment1.getEnd() - segment2.getSegStart()).lensq()
                   ? Segment(segment1.getSegStart(), segment2.getSegStart())
                   : Segment(segment1.getEnd(), segment2.getSegStart());
    }
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
    if (segment1.toVector().lensq() > segment2.toVector().lensq())
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

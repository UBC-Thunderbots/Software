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

bool contains(const LegacyTriangle &out, const Vector &in)
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

bool contains(const Circle &out, const Vector &in)
{
    return distsq(out.getOrigin(), in) <= out.getRadius() * out.getRadius();
}

bool contains(const Circle &out, const Segment &in)
{
    return dist(in, out.getOrigin()) < out.getRadius();
}

bool contains(const Segment &out, const Vector &in)
{
    if (collinear(in, out.getSegStart(), out.getEnd()))
    {
        // if collinear we only need to check one of the coordinates,
        // arbitrarily choose x
        return (in.x() <= out.getSegStart().x() && in.x() >= out.getEnd().x()) ||
               (in.x() <= out.getEnd().x() && in.x() >= out.getSegStart().x());
    }

    return false;
}

bool contains(const Ray &out, const Vector &in)
{
    Point point_in_ray_direction = out.getRayStart() + out.getDirection();
    if (collinear(in, out.getRayStart(), point_in_ray_direction) &&
        (in - out.getRayStart()).norm() == out.getDirection().norm())
    {
        return true;
    }
    return false;
}

bool contains(const Rectangle &out, const Vector &in)
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

std::vector<std::pair<Vector, Angle>> angleSweepCirclesAll(
    const Vector &src, const Vector &p1, const Vector &p2,
    const std::vector<Point> &obstacles, const double &radius)
{
    std::vector<std::pair<Vector, Angle>> ret;

    const Angle offangle = (p1 - src).orientation();
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
                return ret;
            }
        }
        ret.push_back(std::make_pair(collinear_seg.toVector(), Angle::zero()));
        return ret;
    }

    std::vector<std::pair<Angle, int>> events;
    events.reserve(2 * obstacles.size() + 2);
    events.push_back(std::make_pair(Angle::zero(), 1));  // p1 becomes angle 0
    events.push_back(
        std::make_pair(((p2 - src).orientation() - offangle).angleMod(), -1));
    for (Vector i : obstacles)
    {
        Vector diff = i - src;
        if (diff.len() < radius)
        {
            return ret;
        }

        const Angle cent   = (diff.orientation() - offangle).angleMod();
        const Angle span   = Angle::asin(radius / diff.len());
        const Angle range1 = cent - span;
        const Angle range2 = cent + span;

        if (range1 < -Angle::half() || range2 > Angle::half())
        {
            continue;
        }
        events.push_back(std::make_pair(range1, -1));
        events.push_back(std::make_pair(range2, 1));
    }
    // do angle sweep for largest angle
    std::sort(events.begin(), events.end());
    Angle sum   = Angle::zero();
    Angle start = events[0].first;
    int cnt     = 0;
    for (std::size_t i = 0; i + 1 < events.size(); ++i)
    {
        cnt += events[i].second;
        assert(cnt <= 1);
        if (cnt > 0)
        {
            sum += events[i + 1].first - events[i].first;
        }
        else
        {
            const Angle mid    = start + sum / 2 + offangle;
            const Vector ray   = Vector::createFromAngle(mid) * 10.0;
            const Vector inter = lineIntersection(src, src + ray, p1, p2).value();

            ret.push_back(std::make_pair(inter, sum));

            sum   = Angle::zero();
            start = events[i + 1].first;
        }
    }
    return ret;
}

std::pair<Vector, Angle> angleSweepCircles(const Vector &src, const Vector &p1,
                                           const Vector &p2,
                                           const std::vector<Vector> &obstacles,
                                           const double &radius)
{
    // default value to return if nothing is valid
    Vector bestshot      = (p1 + p2) * 0.5;
    const Angle offangle = (p1 - src).orientation();
    if (collinear(src, p1, p2))
    {
        return std::make_pair(bestshot, Angle::zero());
    }
    std::vector<std::pair<Angle, int>> events;
    events.reserve(2 * obstacles.size() + 2);
    events.push_back(std::make_pair(Angle::zero(), 1));  // p1 becomes angle 0
    events.push_back(
        std::make_pair(((p2 - src).orientation() - offangle).angleMod(), -1));
    for (Vector i : obstacles)
    {
        Vector diff = i - src;
        if (diff.len() < radius)
        {
            return std::make_pair(bestshot, Angle::zero());
        }
        const Angle cent   = (diff.orientation() - offangle).angleMod();
        const Angle span   = Angle::asin(radius / diff.len());
        const Angle range1 = cent - span;
        const Angle range2 = cent + span;

        if (range1 < -Angle::half() || range2 > Angle::half())
        {
            continue;
        }
        events.push_back(std::make_pair(range1, -1));
        events.push_back(std::make_pair(range2, 1));
    }
    // do angle sweep for largest angle
    std::sort(events.begin(), events.end());
    Angle best  = Angle::zero();
    Angle sum   = Angle::zero();
    Angle start = events[0].first;
    int cnt     = 0;
    for (std::size_t i = 0; i + 1 < events.size(); ++i)
    {
        cnt += events[i].second;
        assert(cnt <= 1);
        if (cnt > 0)
        {
            sum += events[i + 1].first - events[i].first;
            if (best < sum)
            {
                best = sum;
                // shoot ray from point p
                // intersect with line p1-p2
                const Angle mid    = start + sum / 2 + offangle;
                const Vector ray   = Vector::createFromAngle(mid) * 10.0;
                const Vector inter = lineIntersection(src, src + ray, p1, p2).value();
                bestshot           = inter;
            }
        }
        else
        {
            sum   = Angle::zero();
            start = events[i + 1].first;
        }
    }
    return std::make_pair(bestshot, best);
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

Angle vertexAngle(Vector a, Vector b, Vector c)
{
    return ((a - b).orientation() - (c - b).orientation()).angleMod();
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

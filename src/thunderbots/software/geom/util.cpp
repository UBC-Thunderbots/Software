#include "geom/util.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "geom/angle.h"


double proj_len(const Seg &first, const Vector &second)
{
    return proj_len(first.toVector(), second - first.start);
}

double proj_len(const Vector &first, const Vector &second)
{
    return first.dot(second) / first.len();
}

double dist(const Vector &first, const Vector &second)
{
    return (first - second).len();
}
double dist(const Seg &first, const Seg &second)
{
    if (intersects(first, second))
    {
        return 0.0;
    }
    return std::sqrt(
        std::min(std::min(distsq(first, second.start), distsq(first, second.end)),
                 std::min(distsq(second, first.start), distsq(second, first.end))));
}

double dist(const Line &first, const Vector &second)
{
    if (isDegenerate(first))
    {
        return dist(first.first, second);
    }
    return fabs((second - first.first).cross(first.second - first.first) /
                (first.second - first.first).len());
}

double dist(const Vector &first, const Line &second)
{
    return dist(second, first);
}

double dist(const Vector &first, const Seg &second)
{
    return std::sqrt(distsq(first, second));
}

double dist(const Seg &first, const Vector &second)
{
    return dist(second, first);
}

double distsq(const Vector &first, const Seg &second)
{
    double seglensq    = lensq(second);
    Vector relsecond_s = first - second.start;
    Vector relsecond_e = first - second.end;

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

    double lensq_s = distsq(second.start, first), lensq_e = distsq(second.end, first);

    return std::min(lensq_s, lensq_e);
}

double distsq(const Seg &first, const Vector &second)
{
    return distsq(second, first);
}

double distsq(const Vector &first, const Vector &second)
{
    return (first - second).lensq();
}

bool isDegenerate(const Seg &seg)
{
    return distsq(seg.start, seg.end) < EPS2;
}

bool isDegenerate(const Line &line)
{
    return distsq(line.first, line.second) < EPS2;
}

bool isDegenerate(const Ray &ray)
{
    return distsq(ray.start, ray.dir) < EPS2;
}

double len(const Seg &seg)
{
    return dist(seg.start, seg.end);
}

double lensq(const Seg &seg)
{
    return distsq(seg.start, seg.end);
}

double lensq(const Line &line)
{
    (void)line;  // unused
    return std::numeric_limits<double>::infinity();
}

bool contains(const Triangle &out, const Vector &in)
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
    return distsq(out.origin, in) <= out.radius * out.radius;
}

bool contains(const Circle &out, const Seg &in)
{
    return dist(in, out.origin) < out.radius;
}

bool contains(const Seg &out, const Vector &in)
{
    if (collinear(in, out.start, out.end))
    {
        // if collinear we only need to check one of the coordinates,
        // arbitrarily choose x
        return (in.x() <= out.start.x() && in.x() >= out.end.x()) ||
               (in.x() <= out.end.x() && in.x() >= out.start.x());
    }

    return false;
}

bool contains(const Ray &out, const Vector &in)
{
    if (collinear(in, out.start, out.dir))
    {
        return sign(in.x() - out.start.x()) == sign(out.dir.x() - out.start.x()) &&
               sign(in.y() - out.start.y()) == sign(out.dir.y() - out.start.y());
    }

    return false;
}

bool contains(const Rect &out, const Vector &in)
{
    return out.containsPoint(in);
}

bool intersects(const Triangle &first, const Circle &second)
{
    return contains(first, second.origin) ||
           dist(getSide(first, 0), second.origin) < second.radius ||
           dist(getSide(first, 1), second.origin) < second.radius ||
           dist(getSide(first, 2), second.origin) < second.radius;
}
bool intersects(const Circle &first, const Triangle &second)
{
    return intersects(second, first);
}

bool intersects(const Circle &first, const Circle &second)
{
    return (first.origin - second.origin).len() < (first.radius + second.radius);
}

bool intersects(const Ray &first, const Seg &second)
{
    auto isect = lineIntersection(first.start, first.dir, second.start, second.end);
    if (isect.has_value())
    {
        return contains(first, isect.value()) && contains(second, isect.value());
    }
    return collinear(first.start, first.dir, second.start);
}
bool intersects(const Seg &first, const Ray &second)
{
    return intersects(second, first);
}

bool intersects(const Seg &first, const Circle &second)
{
    // if the segment is inside the circle AND at least one of the points is
    // outside the circle
    return contains(second, first) &&
           (distsq(first.start, second.origin) > second.radius * second.radius ||
            distsq(first.end, second.origin) > second.radius * second.radius);
}
bool intersects(const Circle &first, const Seg &second)
{
    return intersects(second, first);
}

bool intersects(const Seg &first, const Seg &second)
{
    if (sign((first.start - first.end).cross(second.start - second.end)) == 0)
    {
        // find distance of two endpoints on segments furthest away from each
        // other
        double mx_len = std::sqrt(std::max(std::max((second.start - first.end).lensq(),
                                                    (second.end - first.end).lensq()),
                                           std::max((second.start - first.start).lensq(),
                                                    (second.end - first.start).lensq())));
        // if the segments cross then this distance should be less than
        // the sum of the distances of the line segments
        return mx_len <
               (first.start - first.end).len() + (second.start - second.end).len() + EPS;
    }

    return sign((first.end - first.start).cross(second.start - first.start)) *
                   sign((first.end - first.start).cross(second.end - first.start)) <=
               0 &&
           sign((second.end - second.start).cross(first.start - second.start)) *
                   sign((second.end - second.start).cross(first.end - second.start)) <=
               0;
}

template <size_t N>
Vector getVertex(const Poly<N> &poly, unsigned int i)
{
    if (i > N)
        throw std::out_of_range("poly does not have that many sides!!!");
    else
        return poly[i];
}

template <size_t N>
void setVertex(Poly<N> &poly, unsigned int i, const Vector &v)
{
    if (i > N)
        throw std::out_of_range("poly does not have that many sides!!!");
    else
        poly[i] = v;
}

template <size_t N>
Seg getSide(const Poly<N> &poly, unsigned int i)
{
    return Seg(getVertex(poly, i), getVertex(poly, (i + 1) % N));
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
        Seg collinear_seg = Seg(src, p1);
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

Vector clipPoint(const Vector &p, const Rect &r)
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

std::vector<Vector> lineRectIntersect(const Rect &r, const Vector &segA,
                                      const Vector &segB)
{
    std::vector<Vector> ans;
    for (unsigned int i = 0; i < 4; i++)
    {
        const Vector &a = r[i];
        // to draw a line segment from point 3 to point 0
        const Vector &b = r[(i + 1) % 4];
        if (intersects(Seg(a, b), Seg(segA, segB)) &&
            uniqueLineIntersects(a, b, segA, segB))
        {
            ans.push_back(lineIntersection(a, b, segA, segB).value());
        }
    }
    return ans;
}

Vector vectorRectIntersect(const Rect &r, const Vector &vecA, const Vector &vecB)
{
    std::vector<Vector> points = lineRectIntersect(r, vecA, (vecB - vecA) * 100 + vecA);
    for (Vector i : points)
    {
        if (contains(Ray(vecA, vecB), i))
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
}

bool uniqueLineIntersects(const Vector &a, const Vector &b, const Vector &c,
                          const Vector &d)
{
    return std::abs((d - c).cross(b - a)) > EPS;
}

std::vector<Point> lineIntersection(const Seg &a, const Seg &b)
{
    if (std::fabs((b.end - b.start).cross(a.end - a.start)) < EPS)
    {
        // parallel line segments, find if they're collinear and return the 2 points
        // on the line they both lay on if they are collinear and intersecting
        // shamelessly copypasted from
        // https://stackoverflow.com/questions/22456517/algorithm-for-finding-the-segment-overlapping-two-collinear-segments
        if (collinear(a.start, b.start, b.end) && collinear(a.end, b.start, b.end))
        {
            double slope      = (a.end.y() - a.start.y()) / (a.end.x() - a.start.x());
            bool isHorizontal = slope < EPS;
            bool isDescending = slope < 0 && !isHorizontal;
            double invertY    = isDescending || isHorizontal ? -1 : 1;

            Point min1 = Point(std::min(a.start.x(), a.end.x()),
                               std::min(a.start.y() * invertY, a.end.y() * invertY));
            Point max1 = Point(std::max(a.start.x(), a.end.x()),
                               std::max(a.start.y() * invertY, a.end.y() * invertY));

            Point min2 = Point(std::min(b.start.x(), b.end.x()),
                               std::min(b.start.y() * invertY, b.end.y() * invertY));
            Point max2 = Point(std::max(b.start.x(), b.end.x()),
                               std::max(b.start.y() * invertY, b.end.y() * invertY));

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

    return std::vector<Point>{a.start +
                              (a.start - b.start).cross(b.end - b.start) /
                                  (b.end - b.start).cross(a.end - a.start) *
                                  (a.end - a.start)};
}

// shamelessly copy-pasted from RoboJackets
std::optional<Point> lineIntersection(const Vector &a, const Vector &b, const Vector &c,
                                      const Vector &d)
{
    Seg line1(a, b), line2(c, d);
    double x1 = line1.start.x();
    double y1 = line1.start.y();
    double x2 = line1.end.x();
    double y2 = line1.end.y();
    double x3 = line2.start.x();
    double y3 = line2.start.y();
    double x4 = line2.end.x();
    double y4 = line2.end.y();

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

    return n.dot(p - x0);
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

bool pointInFrontVector(Vector offset, Vector dir, Vector p)
{
    // compare angle different
    Angle a1   = dir.orientation();
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
        double perpDist =
            std::sqrt(circle.radius * circle.radius - (circle.origin - start).lensq());
        Point p1 = start + (circle.origin - start).perp().norm(perpDist + buffer);
        Point p2 = start - (circle.origin - start).perp().norm(perpDist + buffer);
        return std::make_pair(p1, p2);
    }
    else
    {
        double radiusAngle = std::acos(circle.radius / (start - circle.origin).len());
        Point p1           = circle.origin +
                   (start - circle.origin)
                       .rotate(Angle::ofRadians(radiusAngle))
                       .norm(circle.radius + buffer);
        Point p2 = circle.origin +
                   (start - circle.origin)
                       .rotate(-Angle::ofRadians(radiusAngle))
                       .norm(circle.radius + buffer);
        return std::make_pair(p1, p2);
    }
}

bool pointIsRightOfLine(const Seg &line, const Point &point)
{
    return (line.end.x() - line.start.x()) * (point.y() - line.start.y()) -
               (line.end.y() - line.start.y()) * (point.x() - line.start.x()) <
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

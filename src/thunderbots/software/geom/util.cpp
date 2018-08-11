#include "geom/util.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "geom/angle.h"


double proj_len(const Seg &first, const Vector2 &second)
{
    return proj_len(first.to_vector2(), second - first.start);
}

double proj_len(const Vector2 &first, const Vector2 &second)
{
    return first.dot(second) / first.len();
}

double dist(const Vector2 &first, const Vector2 &second)
{
    return (first - second).len();
}
double dist(const Seg &first, const Seg &second)
{
    if (intersects(first, second))
    {
        return 0.0;
    }
    return std::sqrt(std::min(
        std::min(distsq(first, second.start), distsq(first, second.end)),
        std::min(distsq(second, first.start), distsq(second, first.end))));
}

double dist(const Line &first, const Vector2 &second)
{
    if (is_degenerate(first))
    {
        return dist(first.first, second);
    }
    return fabs(
        (second - first.first).cross(first.second - first.first) /
        (first.second - first.first).len());
}

double dist(const Vector2 &first, const Line &second)
{
    return dist(second, first);
}

double dist(const Vector2 &first, const Seg &second)
{
    return std::sqrt(distsq(first, second));
}

double dist(const Seg &first, const Vector2 &second)
{
    return dist(second, first);
}

double distsq(const Vector2 &first, const Seg &second)
{
    double seglensq     = lensq(second);
    Vector2 relsecond_s = first - second.start;
    Vector2 relsecond_e = first - second.end;

    Vector2 s_vec2 = second.to_vector2();

    if (s_vec2.dot(relsecond_s) > 0 &&
        second.reverse().to_vector2().dot(relsecond_e) > 0)
    {
        if (is_degenerate(second))
        {
            return relsecond_s.len();
        }
        double cross = relsecond_s.cross(s_vec2);
        return std::fabs(cross * cross / seglensq);
    }

    double lensq_s = distsq(second.start, first),
           lensq_e = distsq(second.end, first);

    return std::min(lensq_s, lensq_e);
}

double distsq(const Seg &first, const Vector2 &second)
{
    return distsq(second, first);
}

double distsq(const Vector2 &first, const Vector2 &second)
{
    return (first - second).lensq();
}

bool is_degenerate(const Seg &seg)
{
    return distsq(seg.start, seg.end) < EPS2;
}

bool is_degenerate(const Line &line)
{
    return distsq(line.first, line.second) < EPS2;
}

bool is_degenerate(const Ray &ray)
{
    return distsq(ray.start, ray.dir) < EPS2;
}

double len(const Seg &seg)
{
    return dist(seg.start, seg.end);
}

double len(const Line &line)
{
    (void)line;  // unused
    return std::numeric_limits<double>::infinity();
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

bool contains(const Triangle &out, const Vector2 &in)
{
    double angle = 0;
    for (int i = 0, j = 2; i < 3; j = i++)
    {
        if ((in - out[i]).len() < EPS)
        {
            return true;  // SPECIAL CASE
        }
        double a = atan2(
            (out[i] - in).cross(out[j] - in), (out[i] - in).dot(out[j] - in));
        angle += a;
    }
    return std::fabs(angle) > 6;
}

bool contains(const Circle &out, const Vector2 &in)
{
    return distsq(out.origin, in) <= out.radius * out.radius;
}

bool contains(const Circle &out, const Seg &in)
{
    return dist(in, out.origin) < out.radius;
}

bool contains(const Seg &out, const Vector2 &in)
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

bool contains(const Ray &out, const Vector2 &in)
{
    if (collinear(in, out.start, out.dir))
    {
        return sign(in.x() - out.start.x()) == sign(out.dir.x() - out.start.x()) &&
               sign(in.y() - out.start.y()) == sign(out.dir.y() - out.start.y());
    }
    
    return false;
}

bool contains(const Rect &out, const Vector2 &in)
{
    return out.point_inside(in);
}

bool intersects(const Triangle &first, const Circle &second)
{
    return contains(first, second.origin) ||
           dist(get_side(first, 0), second.origin) < second.radius ||
           dist(get_side(first, 1), second.origin) < second.radius ||
           dist(get_side(first, 2), second.origin) < second.radius;
}
bool intersects(const Circle &first, const Triangle &second)
{
    return intersects(second, first);
}

bool intersects(const Circle &first, const Circle &second)
{
    return (first.origin - second.origin).len() <
           (first.radius + second.radius);
}

bool intersects(const Ray &first, const Seg &second)
{
    if (std::abs(first.to_vector2().cross(second.to_vector2())) > EPS)
    {
        Vector2 isect =
            line_intersect(first.start, first.dir, second.start, second.end);
        return contains(first, isect) && contains(second, isect);
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
           (distsq(first.start, second.origin) >
                second.radius * second.radius ||
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
        double mx_len = std::sqrt(std::max(
            std::max(
                (second.start - first.end).lensq(),
                (second.end - first.end).lensq()),
            std::max(
                (second.start - first.start).lensq(),
                (second.end - first.start).lensq())));
        // if the segments cross then this distance should be less than
        // the sum of the distances of the line segments
        return mx_len < (first.start - first.end).len() +
                            (second.start - second.end).len() + EPS;
    }

    return sign((first.end - first.start).cross(second.start - first.start)) *
                   sign((first.end - first.start)
                            .cross(second.end - first.start)) <=
               0 &&
           sign((second.end - second.start).cross(first.start - second.start)) *
                   sign((second.end - second.start)
                            .cross(first.end - second.start)) <=
               0;
}

template <size_t N>
Vector2 get_vertex(const Poly<N> &poly, unsigned int i)
{
    return poly[i % N];
}

template <size_t N>
void set_vertex(Poly<N> &poly, unsigned int i, const Vector2 &v)
{
    poly[i % N] = v;
}

template <size_t N>
Seg get_side(const Poly<N> &poly, unsigned int i)
{
    return Seg(get_vertex(poly, i), get_vertex(poly, i + 1));
}

std::vector<std::pair<Vector2, Angle>> angle_sweep_circles_all(
    const Vector2 &src, const Vector2 &p1, const Vector2 &p2,
    const std::vector<Vector2> &obstacles, const double &radius)
{
    std::vector<std::pair<Vector2, Angle>> ret;

    const Angle offangle = (p1 - src).orientation();
    if (collinear(src, p1, p2))
    {
        // std::cerr << "geom: collinear " << src << " " << p1 << " " << p2 <<
        // std::endl;
        // std::cerr << (p1 - src) << " " << (p2 - src) << std::endl;
        // std::cerr << (p1 - src).cross(p2 - src) << std::endl;
        return ret;
    }

    std::vector<std::pair<Angle, int>> events;
    events.reserve(2 * obstacles.size() + 2);
    events.push_back(std::make_pair(Angle::zero(), 1));  // p1 becomes angle 0
    events.push_back(
        std::make_pair(((p2 - src).orientation() - offangle).angle_mod(), -1));
    for (Vector2 i : obstacles)
    {
        Vector2 diff = i - src;
        if (diff.len() < radius)
        {
            return ret;
        }

        const Angle cent   = (diff.orientation() - offangle).angle_mod();
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
            const Angle mid     = start + sum / 2 + offangle;
            const Vector2 ray   = Vector2::of_angle(mid) * 10.0;
            const Vector2 inter = line_intersect(src, src + ray, p1, p2);

            ret.push_back(std::make_pair(inter, sum));

            sum   = Angle::zero();
            start = events[i + 1].first;
        }
    }
    return ret;
}

std::pair<Vector2, Angle> angle_sweep_circles(
    const Vector2 &src, const Vector2 &p1, const Vector2 &p2,
    const std::vector<Vector2> &obstacles, const double &radius)
{
    // default value to return if nothing is valid
    Vector2 bestshot     = (p1 + p2) * 0.5;
    const Angle offangle = (p1 - src).orientation();
    if (collinear(src, p1, p2))
    {
        // std::cerr << "geom: collinear " << src << " " << p1 << " " << p2 <<
        // std::endl;
        // std::cerr << (p1 - src) << " " << (p2 - src) << std::endl;
        // std::cerr << (p1 - src).cross(p2 - src) << std::endl;
        return std::make_pair(bestshot, Angle::zero());
    }
    std::vector<std::pair<Angle, int>> events;
    events.reserve(2 * obstacles.size() + 2);
    events.push_back(std::make_pair(Angle::zero(), 1));  // p1 becomes angle 0
    events.push_back(
        std::make_pair(((p2 - src).orientation() - offangle).angle_mod(), -1));
    for (Vector2 i : obstacles)
    {
        Vector2 diff = i - src;
        if (diff.len() < radius)
        {
            return std::make_pair(bestshot, Angle::zero());
        }
        const Angle cent   = (diff.orientation() - offangle).angle_mod();
        const Angle span   = Angle::asin(radius / diff.len());
        const Angle range1 = cent - span;
        const Angle range2 = cent + span;

        /*
           if (range1 < -M_PI) {
           // [-PI, range2]
           events.push_back(std::make_pair(-M_PI, -1));
           events.push_back(std::make_pair(range2, 1));
           // [range1, PI]
           events.push_back(std::make_pair(range1 + 2 * M_PI, -1));
           events.push_back(std::make_pair(M_PI, 1));
           } else if (range2 > M_PI) {
           // [range1, PI]
           events.push_back(std::make_pair(range1, -1));
           events.push_back(std::make_pair(M_PI, 1));
           // [-PI, range2]
           events.push_back(std::make_pair(-M_PI, -1));
           events.push_back(std::make_pair(range2 - 2 * M_PI, 1));
           } else {
           events.push_back(std::make_pair(range1, -1));
           events.push_back(std::make_pair(range2, 1));
           }
         */

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
                const Angle mid     = start + sum / 2 + offangle;
                const Vector2 ray   = Vector2::of_angle(mid) * 10.0;
                const Vector2 inter = line_intersect(src, src + ray, p1, p2);
                bestshot            = inter;
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

std::vector<Vector2> circle_boundaries(
    const Vector2 &centre, double radius, int num_points)
{
    Angle rotate_amount = Angle::full() / num_points;
    std::vector<Vector2> ans;
    Vector2 bound(radius, 0.0);
    for (int i = 0; i < num_points; i++)
    {
        Vector2 temp = centre + bound;
        ans.push_back(temp);
        bound = bound.rotate(rotate_amount);
    }
    return ans;
}

bool collinear(const Vector2 &a, const Vector2 &b, const Vector2 &c)
{
    if ((a - b).lensq() < EPS2 || (b - c).lensq() < EPS2 ||
        (a - c).lensq() < EPS2)
    {
        return true;
    }
    return std::fabs((b - a).cross(c - a)) < EPS;
}

Vector2 clip_point(
    const Vector2 &p, const Vector2 &bound1, const Vector2 &bound2)
{
    const double minx = std::min(bound1.x(), bound2.x());
    const double miny = std::min(bound1.y(), bound2.y());
    const double maxx = std::max(bound1.x(), bound2.x());
    const double maxy = std::max(bound1.y(), bound2.y());
    Vector2 ret       = p;
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

Vector2 clip_point(const Vector2 &p, const Rect &r)
{
    const double minx = r.sw_corner().x();
    const double miny = r.sw_corner().y();
    const double maxx = r.ne_corner().x();
    const double maxy = r.ne_corner().y();
    Vector2 ret       = p;
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

// TODO: does this work? unit test
std::vector<Vector2> line_circle_intersect(
    const Vector2 &centre, double radius, const Vector2 &segA,
    const Vector2 &segB)
{
    std::vector<Vector2> ans;

    // take care of 0 length segments too much error here
    if ((segB - segA).lensq() < EPS)
    {
        return ans;
    }

    double lenseg = (segB - segA).dot(centre - segA) / (segB - segA).len();
    Vector2 C     = segA + lenseg * (segB - segA).norm();

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

std::vector<Vector2> line_rect_intersect(
    const Rect &r, const Vector2 &segA, const Vector2 &segB)
{
    std::vector<Vector2> ans;
    for (unsigned int i = 0; i < 4; i++)
    {
        const Vector2 &a = r[i];
        const Vector2 &b = r[i + 1];
        if (intersects(Seg(a, b), Seg(segA, segB)) &&
            unique_line_intersect(a, b, segA, segB))
        {
            ans.push_back(line_intersect(a, b, segA, segB));
        }
    }
    return ans;
}

Vector2 vector_rect_intersect(
    const Rect &r, const Vector2 &vecA, const Vector2 &vecB)
{
    /*std::cout << vecA << vecB << r.ne_corner() << r.sw_corner();
       for (unsigned int i = 0; i < 4; i++) {
        unsigned int j = (i + 1)%4;
        const Vector2 &a = r[i];
        const Vector2 &b = r[j];
        if ( intersects(Ray(vecA, vecB), Seg( a, b ) )) {
            Vector2 intersect = line_intersect(a, b, vecA, vecB);
            std::cout << std::endl;
            return intersect;
        }
       }
       std::cout << "fail \n";
       return r.centre();  // return the center of the rectangle, if no valid
       answer is found*/
    std::vector<Vector2> points =
        line_rect_intersect(r, vecA, (vecB - vecA) * 100 + vecA);
    for (Vector2 i : points)
    {
        if (contains(Ray(vecA, vecB), i))
        {
            return i;
        }
    }
    return Vector2(
        1.0 / 0.0, 1.0 / 0.0);  // no solution found, propagate infinity
}

Vector2 closest_lineseg_point(
    const Vector2 &centre, const Vector2 &segA, const Vector2 &segB)
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
    Vector2 C     = segA + lenseg * (segB - segA).norm();

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
// this function is never used
std::vector<Vector2> lineseg_circle_intersect(
    const Vector2 &centre, double radius, const Vector2 &segA,
    const Vector2 &segB)
{
    std::vector<Vector2> ans;
    std::vector<Vector2> poss =
        line_circle_intersect(centre, radius, segA, segB);

    for (Vector2 i : poss)
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

bool unique_line_intersect(
    const Vector2 &a, const Vector2 &b, const Vector2 &c, const Vector2 &d)
{
    return std::abs((d - c).cross(b - a)) > EPS;
}

std::vector<Point> line_intersect(const Seg &a, const Seg &b)
{
    if (std::fabs((b.end - b.start).cross(a.end - a.start)) < EPS)
    {
        // LOG_WARN(u8"Cross product problem again in new function");
        return std::vector<Point>();
    }

    return std::vector<Point>{a.start +
                              (a.start - b.start).cross(b.end - b.start) /
                                  (b.end - b.start).cross(a.end - a.start) *
                                  (a.end - a.start)};
}

// ported code
Vector2 line_intersect(
    const Vector2 &a, const Vector2 &b, const Vector2 &c, const Vector2 &d)
{
    // TODO figure out why this is asserting
    // assert(std::abs((d - c).cross(b - a)) > EPS);
    if (std::fabs((d - c).cross(b - a)) < EPS)
    {
        // LOG_WARN(u8"Cross product problem again");
    }

    return a + (a - c).cross(d - c) / (d - c).cross(b - a) * (b - a);

    //	// testing new code from
    // http://flassari.is/2008/11/line-line-intersection-in-cplusplus/
    //	// Store the values for fast access and easy
    //	// equations-to-code conversion
    //	float x1 = a.x(), x2 = b.x(), x3 = c.x(), x4 = d.x();
    //	float y1 = a.y(), y2 = b.y(), y3 = c.y(), y4 = d.y();
    //
    //	float dd = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    //	// If d is zero, there is no intersection
    //	if (d == 0) return std::vector<Point>();
    //
    //	// Get the x and y
    //	float pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
    //	float x = ( pre * (x3 - x4) - (x1 - x2) * post ) / dd;
    //	float y = ( pre * (y3 - y4) - (y1 - y2) * post ) / dd;
    //
    //	// Check if the x and y coordinates are within both lines
    //	if ( x < std::min(x1, x2) || x > std::max(x1, x2) ||
    //	x < std::min(x3, x4) || x > std::max(x3, x4) ) return NULL;
    //	if ( y < std::min(y1, y2) || y > std::max(y1, y2) ||
    //	y < std::min(y3, y4) || y > std::max(y3, y4) ) return NULL;
    //
    //	// Return the point of intersection
    //	Point* ret = new Point();
    //	ret->x = x;
    //	ret->y = y;
    //	return ret;
}

// TODO: a line intersect that takes segments would be nice

Vector2 reflect(const Vector2 &v, const Vector2 &n)
{
    if (n.len() < EPS)
    {
        
        return v;
    }
    Vector2 normal = n.norm();
    return v - 2 * v.dot(normal) * normal;
}

Vector2 reflect(const Vector2 &a, const Vector2 &b, const Vector2 &p)
{
    // Make a as origin.
    // Rotate by 90 degrees, does not matter which direction?
    Vector2 n = (b - a).rotate(Angle::quarter());
    return a + reflect(p - a, n);
}

// ported code
Vector2 calc_block_cone(
    const Vector2 &a, const Vector2 &b, const double &radius)
{
    if (a.len() < EPS || b.len() < EPS)
    {
        
    }
    // unit vector and bisector
    Vector2 au = a / a.len();
    Vector2 c  = au + b / b.len();
    // use similar triangle
    return c * (radius / std::fabs(au.cross(c)));
}

Vector2 calc_block_cone(
    const Vector2 &a, const Vector2 &b, const Vector2 &p, const double &radius)
{
    /*
       Vector2 R = p + calc_block_cone(a - p, b - p, radius);
       const double MIN_X = std::min(-2.5, (p.x() + 3.025) / 2.0 - 3.025);
       if (R.x() < MIN_X){
       R = (R - p) * ((MIN_X - p.x()) / (R.x() - p.x())) + p;
       }
       return R;
     */
    return p + calc_block_cone(a - p, b - p, radius);
}

// ported code
Vector2 calc_block_other_ray(
    const Vector2 &a, const Vector2 &c, const Vector2 &g)
{
    return reflect(c - a, g - c);  // this, and the next two instances, were
                                   // changed from a - c since reflect() was
                                   // fixed
}

// ported code
bool goalie_block_goal_post(
    const Vector2 &a, const Vector2 &b, const Vector2 &c, const Vector2 &g)
{
    Vector2 R = reflect(c - a, g - c);
    return fabs(R.cross(b - c)) < EPS;
}

// ported code
Vector2 calc_block_cone_defender(
    const Vector2 &a, const Vector2 &b, const Vector2 &c, const Vector2 &g,
    const double &r)
{
    Vector2 R = reflect(c - a, g - c);
    // std::cout << (R + c) << std::endl;
    return calc_block_cone(R + c, b, c, r);
}

// ported cm code below

double offset_to_line(Vector2 x0, Vector2 x1, Vector2 p)
{
    Vector2 n;

    // get normal to line
    n = (x1 - x0).perp().norm();

    return n.dot(p - x0);
}

double offset_along_line(Vector2 x0, Vector2 x1, Vector2 p)
{
    Vector2 n, v;

    // get normal to line
    n = x1 - x0;
    n = n.norm();

    v = p - x0;

    return n.dot(v);
}

Vector2 segment_near_line(Vector2 a0, Vector2 a1, Vector2 b0, Vector2 b1)
{
    Vector2 v, n, p;
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

Vector2 intersection(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2)
{
    Vector2 a = a2 - a1;

    Vector2 b1r = (b1 - a1).rotate(-a.orientation());
    Vector2 b2r = (b2 - a1).rotate(-a.orientation());
    Vector2 br  = (b1r - b2r);

    return Vector2(b2r.x() - b2r.y() * (br.x() / br.y()), 0.0).rotate(a.orientation()) +
           a1;
}

Angle vertex_angle(Vector2 a, Vector2 b, Vector2 c)
{
    return ((a - b).orientation() - (c - b).orientation()).angle_mod();
}

double closest_point_time(Vector2 x1, Vector2 v1, Vector2 x2, Vector2 v2)
{
    Vector2 v = v1 - v2;
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

bool point_in_front_vector(Vector2 offset, Vector2 dir, Vector2 p)
{
    // compare angle different
    Angle a1   = dir.orientation();
    Angle a2   = (p - offset).orientation();
    Angle diff = (a1 - a2).angle_mod();
    return diff < Angle::quarter() && diff > -Angle::quarter();
}

bool is_clockwise(Vector2 v1, Vector2 v2)
{
    if (v1.y() * v2.x() > v1.x() * v2.y())
    {
        return true;
    }
    return false;
}

std::pair<Point, Point> get_circle_tangent_points(
    const Point &start, const Circle &circle, double buffer)
{
    // If the point is already inside the circe arccos won't work so just return
    // the perp points
    if (contains(circle, start))
    {
        double perpDist = std::sqrt(
            circle.radius * circle.radius - (circle.origin - start).lensq());
        Point p1 =
            start + (circle.origin - start).perp().norm(perpDist + buffer);
        Point p2 =
            start - (circle.origin - start).perp().norm(perpDist + buffer);
        return std::make_pair(p1, p2);
    }
    else
    {
        double radiusAngle =
            std::acos(circle.radius / (start - circle.origin).len());
        Point p1 = circle.origin +
                   (start - circle.origin)
                       .rotate(Angle::of_radians(radiusAngle))
                       .norm(circle.radius + buffer);
        Point p2 = circle.origin +
                   (start - circle.origin)
                       .rotate(-Angle::of_radians(radiusAngle))
                       .norm(circle.radius + buffer);
        return std::make_pair(p1, p2);
    }
}

bool point_is_to_right_of_line(const Seg &line, const Point &point)
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

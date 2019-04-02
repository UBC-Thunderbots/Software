#pragma once

#include <cstddef>
#include <vector>

#include "geom/circle.h"
#include "geom/line.h"
#include "geom/point.h"
#include "geom/ray.h"
#include "geom/rectangle.h"
#include "geom/segment.h"

template <size_t N>
using LegacyPolygon       = std::array<Vector, N>;
using LegacyTriangle      = LegacyPolygon<3>;
using LegacyQuadrilateral = LegacyPolygon<4>;

constexpr double EPS = 1e-9;

constexpr double EPS2 = EPS * EPS;

constexpr int sign(double n)
{
    return n > EPS ? 1 : (n < -EPS ? -1 : 0);
}

inline LegacyTriangle triangle(const Point &a, const Point &b, const Point &c)
{
    return {a, b, c};
}
inline LegacyQuadrilateral quad(const Point &a, const Point &b, const Point &c,
                                const Point &d)
{
    return {a, b, c, d};
}

/**
 * Signed magnitude of the projection of `second` on `first`
 */
double proj_len(const Vector &first, const Vector &second);

/**
 * Signed magnitude of the projection of `first.start -> second` on `first`
 */
double proj_len(const Segment &first, const Vector &second);

/*
 * The family of `contains` functions determines whether
 * the second parameter is contained, even if partially,
 * inside the first parameter.
 */

bool contains(const LegacyTriangle &out, const Vector &in);
bool contains(const Circle &out, const Vector &in);
bool contains(const Circle &out, const Segment &in);
bool contains(const Ray &out, const Vector &in);
bool contains(const Segment &out, const Vector &in);
bool contains(const Rectangle &out, const Vector &in);

/*
 * The family of `intersects` functions determines whether there
 * exists an intersection between one object and another.
 */

bool intersects(const LegacyTriangle &first, const Circle &second);
bool intersects(const Circle &first, const LegacyTriangle &second);
bool intersects(const Circle &first, const Circle &second);
bool intersects(const Segment &first, const Circle &second);
bool intersects(const Circle &first, const Segment &second);
bool intersects(const Segment &first, const Segment &second);
bool intersects(const Ray &first, const Segment &second);
bool intersects(const Segment &first, const Ray &second);

/*
 * The family of `dist` functions calculates the unsigned distance
 * between one object and another.
 */

double dist(const Point &first, const Point &second);
double dist(const Segment &first, const Segment &second);

double dist(const Point &first, const Segment &second);
double dist(const Segment &first, const Point &second);

double dist(const Line &first, const Point &second);
double dist(const Point &first, const Line &second);

double distsq(const Point &first, const Segment &second);
double distsq(const Segment &first, const Point &second);
double distsq(const Point &first, const Point &second);

bool isDegenerate(const Segment &segment);
bool isDegenerate(const Ray &segment);
bool isDegenerate(const Line &line);

double len(const Segment &segment);

double lensq(const Segment &segment);
double lensq(const Line &line);

template <size_t N>
Vector getVertex(const LegacyPolygon<N> &poly, unsigned int i);
template <size_t N>
void setVertex(LegacyPolygon<N> &poly, unsigned int i, Vector &v);

/**
 * Gets the side that is connected to the vertex with index provided
 * and also connected to the vertex with the next index.
 */
template <size_t N>
Segment getSide(const LegacyPolygon<N> &poly, unsigned int i);

/**
 * Checks if 3 points are collinear.
 *
 * @param a a point
 *
 * @param b a point
 *
 * @param c a point
 *
 * @return true if any two points are within EPS distance to each other. (If
 * any two of three points are within EPS distance of each other, they are
 * essentially the same point and the two points will form the same line.)
 *
 * @return true if the cross product of the two lines formed by the three
 * points are smaller than EPS
 */
bool collinear(const Point &a, const Point &b, const Point &c);

/**
 * Performs an angle sweep.
 * Suppose in this world, all objects are circles of fixed radius.
 * You are at point \p src, and you want to shoot a ray between \p p1 and \p p2.
 * This function calculates the largest open angle interval that you can shoot.
 *
 * @pre \p p1 must be to the right of \p p2.
 * In other words, if there is a counterclockwise ordering, \p p1 is before \p
 * p2 from \p src's point of view.
 *
 * @pre The angle \p p1, \p src, \p p2 must not be greater than 180 degrees.
 *
 * @pre \p src must not be between \p p1 and \p p2.
 *
 * @param src the location where you are standing.
 *
 * @param p1 the location of the right-hand edge of the target area.
 *
 * @param p2 the location of the left-hand edge of the target area.
 *
 * @param obstacles the coordinates of the centres of the obstacles.
 *
 * @param radius the radii of the obstacles.
 *
 * @return the best direction to shoot and the size of the angle centered around
 * that direction that is completely free of obstacles,
 * or <code>(<var>p</var>, 0)</code> for some unspecified <var>p</var> if there
 * is no free path.
 */
std::pair<Point, Angle> angleSweepCircles(const Point &src, const Point &p1,
                                          const Point &p2,
                                          const std::vector<Point> &obstacles,
                                          const double &radius);

/**
 * Gets all angles.
 *
 * @pre The point \p src can't be within the radius of the obstacle
 *
 * @param src the location where you are standing.
 *
 * @param p1 the location of the right-hand edge of the target area.
 *
 * @param p2 the location of the left-hand edge of the target area.
 *
 * @param obstacles the coordinates of the centres of the obstacles.
 *
 * @param radius the radii of the obstacles.
 *
 * @return a vector of all possible pairs of directions and angles to a target area.
 * If lines \p src -> \p p1 and \p src -> \p p2 are collinear, the result will contain a
 * single pair of the direction of \p src -> \p p1 and zero angle if the line is not
 * blocked by an obstacle.
 * Otherwise, an empty vector will be returned.
 * An empty vector is returned if the preconditions aren't satisfied.
 */
std::vector<std::pair<Point, Angle>> angleSweepCirclesAll(
    const Point &src, const Point &p1, const Point &p2,
    const std::vector<Point> &obstacles, const double &radius);

/**
 * returns a list of points that lie on the border of the circle
 */
std::vector<Point> circleBoundaries(const Point &centre, double radius, int num_points);

/**
 * Finds the Point on line segment closest to point.
 *
 * @param centre the point.
 *
 * @param segA one end of the line segment.
 *
 * @param segB the other end of the line segment.
 *
 * @return the Point on line segment closest to centre point.
 */
Point closestPointOnSeg(const Point &p, const Point &segA, const Point &segB);

/**
 * Finds the Point on line closest to point.
 *
 * @param centre the point.
 *
 * @param lineA one point on the line.
 *
 * @param lineB another point on the line segment.
 *
 * @return the Point on line closest to centre point.
 */
Point closestPointOnLine(const Point &p, const Point &lineA, const Point &lineB);

/**
 * Finds the points of intersection between a circle and a line.
 * There may be zero, one, or two such points.
 *
 * @param centre the centre of the circle.
 *
 * @param radius the radius of the circle.
 *
 * @param segA one point on the line.
 *
 * @param segB another point on the line.
 *
 * @return the points of intersection.
 */
std::vector<Point> lineCircleIntersect(const Point &centre, double radius,
                                       const Point &segA, const Point &segB);

/**
 * Finds the points of intersection between a circle and a line.
 * There may be zero, one, or two such points.
 *
 * @param r the rectangle.
 *
 * @param segA one point on the line.
 *
 * @param segB another point on the line.
 *
 * @return the points of intersection.
 */
std::vector<Point> lineRectIntersect(const Rectangle &r, const Point &segA,
                                     const Point &segB);

/**
 * Find where a vector intersect a rectangle
 * return the center of the rectangle if there is no intersection
 *
 * @param r the rectangle.
 *
 * @param segA one point on the line.
 *
 * @param segB another point on the line.
 *
 * @return the points of intersection.
 */
Point vectorRectIntersect(const Rectangle &r, const Point &segA, const Point &segB);

/**
 * Clips a point to a rectangle boundary.
 *
 * @param p the point to clip.
 *
 * @param bound1 one corner of the rectangle.
 *
 * @param bound2 the diagonally-opposite corner of the rectangle.
 *
 * @return the closest point to \p p that lies within the rectangle.
 */
Point clipPoint(const Point &p, const Point &bound1, const Point &bound2);

/**
 * Clips a point to a rectangle boundary.
 *
 * @param p the point to clip.
 *
 * @param r the rectangle.
 *
 * @return the closest point to \p p that lies within the rectangle.
 */
Point clipPoint(const Point &p, const Rectangle &r);

/**
 * Computes whether there is a unique intersection of two lines.
 *
 * @param a a point on the first line.
 *
 * @param b another point on the first line.
 *
 * @param c a point on the second line.
 *
 * @param d another point on the second line.
 *
 * @return whether there is one and only one answer
 */
bool uniqueLineIntersects(const Point &a, const Point &b, const Point &c, const Point &d);

/**
 * Computes the intersection of two lines.
 *
 * @pre The lines must be non-parallel.
 *
 * @param a a point on the first line.
 *
 * @param b another point on the first line.
 *
 * @param c a point on the second line.
 *
 * @param d another point on the second line.
 *
 * @return the point of intersection.
 */
std::optional<Point> lineIntersection(const Point &a, const Point &b, const Point &c,
                                      const Point &d);

/**
 * Computes the intersection of two lines.
 *
 * @pre The lines must be non-parallel.
 *
 * @param a a line segment
 *
 * @param b another line segment

 * @return a vector containing the point of intersection if there is a single point
 *         a vector containing the 2 points of the line segment of the overlap if both
 line segments are collinear
 *         and overlapping
 *         otherwise, an empty vector
 */
std::vector<Point> lineIntersection(const Segment &a, const Segment &b);

/**
 * Reflects a ray incident on origin given the normal of the reflecting plane.
 *
 * @pre the normal vector cannot have length smaller than EPS.
 *
 * @param v the incident ray to reflect.
 *
 * @param n the vector normal to the reflecting plane.
 *
 * @return the reflected ray.
 */
Point reflect(const Point &v, const Point &n);

/**
 * Calculates the intersection of a Ray and Segment
 *
 * @param ray: The point and direction
 * @param segment: Line segment defined by 2 points
 * @return Returns {std::nullopt, std::nullopt} if no intersections exist.
 * Returns {Point, std::nullopt} if a single intersection exists.
 * Returns {Point, Point} if the ray and segment are overlapping, where the points define
 * the line segment of overlap.
 */
std::pair<std::optional<Point>, std::optional<Point>> raySegmentIntersection(
    Ray &ray, Segment &segment);

/**
 * Reflects a point across a line.
 *
 * @param a a point on the line.
 *
 * @param b another point on the line.
 *
 * @param p the point to reflect.
 *
 * @return the reflection of \p p across the line.
 */
Point reflect(const Point &a, const Point &b, const Point &p);

/**
 * Given a cone shooting from the origin, determines the furthest location from
 * the origin, at which to place a circle to block the cone.
 *
 * @pre The cone must have nonzero area.
 *
 * @pre \p b must be counterclockwise of \p a.
 *
 * @param a the starting angle of the cone.
 *
 * @param b the ending angle of the cone.
 *
 * @param radius the radius of the circle with which to block the cone.
 *
 * @return the blocking position.
 */
Point calcBlockCone(const Point &a, const Point &b, const double &radius);

/**
 * Given a cone shooting from a point P, determines the furthest location from
 * P, at which to place a circle to block the cone.
 *
 * @pre The cone must have nonzero area.
 *
 * @pre \p b must be counterclockwise of \p a.
 *
 * @param a the starting angle of the cone.
 *
 * @param b the ending angle of the cone.
 *
 * @param radius the radius of the circle with which to block the cone.
 *
 * @param p the source of the cone.
 *
 * @return the blocking position.
 */
Point calcBlockCone(const Point &a, const Point &b, const Point &p, const double &radius);

/**
 * Used for defender_blocks_goal
 * a = goal post side
 * c = ball position
 * g = goalie position
 * returns the other ray/cone boundary that is not blocked by goalie
 * I.e. if p is return value,
 * then points to the other side of line p-c is not covered by goalie.
 */
Point calcBlockOtherRay(const Point &a, const Point &c, const Point &g);

/*
 * Ported code from CM geom util
 */

/**
 * returns perpendicular offset from line x0-x1 to point p
 */
double offsetToLine(Point x0, Point x1, Point p);

/**
 * returns perpendicular offset from line x0-x1 to point p
 */
double offsetAlongLine(Point x0, Point x1, Point p);

/**
 * returns nearest point on segment a0-a1 to line b0-b1
 */
Point segmentNearLine(Point a0, Point a1, Point b0, Point b1);

/**
 * intersection of two segments?
 */
Point intersection(Point a1, Point a2, Point b1, Point b2);

/**
 * gives counterclockwise angle from <a-b> to <c-b>
 */
Angle vertexAngle(Point a, Point b, Point c);

/**
 * returns time of closest point of approach of two points
 * moving along constant velocity vectors.
 */
double closestPointTime(Point x1, Vector v1, Point x2, Vector v2);

/**
 * found out if a point is in the vector's direction or against it
 * if the point normal to the vector, return false
 *
 * @param offset is the position of the origin of the vector
 *
 * @param dir is the direction of the vector
 *
 * @param p is the point is question
 */
bool pointInFrontVector(Point offset, Point dir, Point p);

/**
 * Returns the circle's tangent points.
 *
 * Returns the points on the circle that form tangent lines with the start point
 */
std::pair<Point, Point> getCircleTangentPoints(const Point &start, const Circle &circle,
                                               double buffer = 0.0);

bool pointIsRightOfLine(const Segment &line, const Point &point);

/**
 * Returns the mean of a list of points
 *
 * @param points the vector of points
 * @return the mean point of points
 */
Point getPointsMean(const std::vector<Point> &points);

/**
 * Returns the variance of a list of Points
 *
 * @param points the vector of points
 * @return the variance of the list of points
 */
double getPointsVariance(const std::vector<Point> &points);

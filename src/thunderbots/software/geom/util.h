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

bool contains(const LegacyTriangle &out, const Point &in);
bool contains(const Circle &out, const Point &in);
bool contains(const Circle &out, const Segment &in);
bool contains(const Ray &out, const Point &in);
bool contains(const Segment &out, const Point &in);
bool contains(const Rectangle &out, const Point &in);

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

/**
 * NOTE: the distance from a point to a rectangle is the closest distance from the point
 * to the edge of the rectangle, or 0 if the point is within the rectangle
 */
double dist(const Point &first, const Rectangle &second);

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
 * @return The best point to aim for in the target area and the size of the open interval
 *         through which a shot from the current point to the best point would go. If no
 *         shot could be found, returns std::nullopt
 */
std::optional<std::pair<Vector, Angle>> angleSweepCircles(
    const Point &src, const Point &p1, const Point &p2,
    const std::vector<Point> &obstacles, const double &radius);

/**
 * Performs an angle sweep.
 * Suppose in this world, all objects are circles of fixed radius.
 * You are at point \p src, and you want to shoot a ray from a point to an area
 * This function calculates the all open angle intervals that you can shoot.
 *
 * @pre The point \p src can't be within the radius of the obstacle
 *
 * @param src the location where the sweep is centered (think center of a clock)
 *
 * @param p1 the location of the right-hand edge of the target area.
 *
 * @param p2 the location of the left-hand edge of the target area.
 *
 * @param obstacles the coordinates of the centres of the obstacles.
 *
 * @param radius the radii of the obstacles.
 *
 * @return A vector of possible (ie. not blocked) pairs of points to shoot to in the
 *         target area,  and the size of the open interval for that point.
 *         ie. for a return point `p` and angle `a`, if `v` is a vector from `src` to `p`,
 *         then the open interval through which we are shooting is from `angle(v) - a/2`
 *         to `angle(v) + a/2`
 *         If lines src -> p1 and src -> p2 are collinear and src -> p1 is not blocked by
 *         an obstacle, the result will contain a single pair of the direction of
 *         src -> p1 and zero angle if the line is not blocked by an obstacle.
 *         An empty vector is returned if the preconditions aren't satisfied.
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
 * Calculates the intersection of two Rays
 *
 * @param ray1: First Ray
 * @param ray2: Second Ray
 * @return Returns std::nullopt if no intersections exist, or if there are infinite
 * intersections (overlapping) Returns Point if a single intersection exists.
 */
std::optional<Point> getRayIntersection(Ray ray1, Ray ray2);

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
 * Calculates the acute angle formed by the two given vectors
 *
 * @param v1
 * @param v2
 *
 * @return The acute angle formed by v1 and v2
 */
Angle acuteVertexAngle(Vector v1, Vector v2);

/**
 * Calculates the acute angle formed by the vector p2->p1 and p2->p3
 *
 * @param p1
 * @param p2
 * @param p3
 *
 * @return the acute angle formed by the vector p2->p1 and p2->p3
 */
Angle acuteVertexAngle(Point p1, Point p2, Point p3);

Angle minAngleBetweenVectors(Vector v1, Vector v2);

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

/**
 * Returns the tangent vectors of a Circle and Point (Vectors are directed towards Circle)
 *
 * @param reference: The point which the tangent vectors will intersect
 * @param circle: The circle to calculate the tangent vectors of
 * @return the mean point of points
 */
std::pair<Ray, Ray> getCircleTangentRays(const Point reference, const Circle circle);

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

/**
 * Function returns the segment defined by the segment between the intersections of two
 * Rays on a segment
 *
 * @param ray1 (Starting point and direction)
 * @param ray2 (Starting point and direction)
 * @param segment (Segment to find segment of intersection upon)
 * @return Segment, the segment defined by the space between two intersecting rays on the
 * segment parameter std::nullopt, if both rays don't intersect the segment, and the
 * segment is not enclosed between the rays Segment, if one ray intersects the segment,
 * but one of the segment parameters extremes are enclosed within the two rays
 */
std::optional<Segment> getIntersectingSegment(Ray ray1, Ray ray2, Segment segment);

/**
 * Function calculates whether the segment parameter is enclosed between the ray
 * parameters. This means the entirety of the segment lays between the rays
 *
 * @param segment : segment parameter to calculate if its definition lies between the rays
 * @param ray1 : Starting point and direction
 * @param ray2 : Starting point an direction
 * @return Segment: Returns the segment parameter if it is completely enclosed between
 * ray1 and ray2.
 *
 * Example of segment being enclosed by rays:
 *
 *        segment
 *     \ *----*  /
 *      \       /
 *  ray1 \     /ray2
 *        *   *
 *
 * Returns std::nullopt of the ray is not completely enclosed between the rays, or
 * not at all
 */
std::optional<Segment> segmentEnclosedBetweenRays(Segment segment, Ray ray1, Ray ray2);

/**
 * Function merges overlapping parallel segments into one combined segment
 *
 * @param segment1 : first segment
 * @param segment2 : second segment
 * @return Segment: Returns the merged segment if segment1 & segment2 are parallel and
 * partially/completely overlapping
 * Returns std::nullopt if the segments aren't parallel or overlapping
 */
std::optional<Segment> mergeOverlappingParallelSegments(Segment segment1,
                                                        Segment segment2);

/**
 * Function calculates if the segment parameters are redundant, for example, if segment2
 * is parallel and contained within segment1
 *
 * @param segment1 : first segment
 * @param segment2 : second segment
 * @return Segment: If the segments are redundant, returns the larger segment
 *         Returns std::nullopt if the segments aren't parallel, arem't overlapping, or
 * aren't redundant
 */
std::optional<Segment> mergeFullyOverlappingSegments(Segment segment1, Segment segment2);

/**
 * Returns the binary trespass score of a point and rectangle
 *
 * @param point The point to check for trespassing
 * @param rectangle The rectangle to check for trespassing by the Point parameter
 * @return 1 if the point exists within the rectangle, or on the boundry of the rectangle
 *         0 if the point exists outside of the rectangle
 */
int calcBinaryTrespassScore(const Rectangle &rectangle, const Point &point);

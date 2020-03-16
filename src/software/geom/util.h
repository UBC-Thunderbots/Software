#pragma once

#include <cstddef>
#include <vector>

#include "software/geom/shot.h"
#include "software/new_geom/circle.h"
#include "software/new_geom/line.h"
#include "software/new_geom/point.h"
#include "software/new_geom/polygon.h"
#include "software/new_geom/ray.h"
#include "software/new_geom/rectangle.h"
#include "software/new_geom/segment.h"
#include "software/new_geom/triangle.h"

constexpr double EPS = 1e-9;

constexpr double EPS2 = EPS * EPS;

constexpr int sign(double n)
{
    return n > EPS ? 1 : (n < -EPS ? -1 : 0);
}

/**
 * Signed magnitude of the projection of `second` on `first`
 */
double proj_length(const Vector &first, const Vector &second);

/**
 * Signed magnitude of the projection of `first.start -> second` on `first`
 */
double proj_length(const Segment &first, const Vector &second);

/*
 * The family of `contains` functions determines whether
 * the second parameter is contained, even if partially,
 * inside the first parameter.
 */
bool contains(const Triangle &out, const Point &in);
bool contains(const Circle &out, const Point &in);
bool contains(const Circle &out, const Segment &in);
bool contains(const Ray &out, const Point &in);
bool contains(const Segment &out, const Point &in);
bool contains(const Rectangle &out, const Point &in);

/*
 * The family of `intersects` functions determines whether there
 * exists an intersection between one object and another.
 */
bool intersects(const Triangle &first, const Circle &second);
bool intersects(const Circle &first, const Triangle &second);
bool intersects(const Circle &first, const Circle &second);
bool intersects(const Segment &first, const Circle &second);
bool intersects(const Circle &first, const Segment &second);
bool intersects(const Segment &first, const Segment &second);
bool intersects(const Ray &first, const Segment &second);
bool intersects(const Segment &first, const Ray &second);

bool isDegenerate(const Segment &segment);
bool isDegenerate(const Ray &segment);

double length(const Segment &segment);

double lengthSquared(const Segment &segment);

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
 * Checks if 2 Segments are collinear.
 *
 * @param segment1 : The first Segment
 *
 * @param segment2 : The second Segment
 *
 * @return true : If the Segment1 and Segment2 are collinear within EPS disance
 *
 * @return false : If Segment1 and Segment2 are NOT collinear within EPS distance
 */
bool collinear(const Segment &segment1, const Segment &segment2);

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
Point closestPointOnSeg(const Point &centre, const Point &segA, const Point &segB);
Point closestPointOnSeg(const Point &p, const Segment &segment);

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
 * Finds the points of intersection between a rectangle and a line.
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
Vector reflect(const Vector &v, const Vector &n);

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
    const Ray &ray, const Segment &segment);

/**
 * Calculates the intersection of a Ray and Rectangle
 *
 * @param ray The ray
 * @param rectangle The rectangle
 * @return Returns {std::nullopt, std::nullopt} if no intersections exist.
 * Returns {Point, std::nullopt} if a single intersection exists.
 * Returns {Point, Point} if the ray overlaps a segment of the rectangle,
 * where the points define the line segment of overlap.
 */
std::pair<std::optional<Point>, std::optional<Point>> rayRectangleIntersection(
    const Ray &ray, const Rectangle &rectangle);

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
Point calcBlockCone(const Vector &a, const Vector &b, const double &radius);

/**
 * Given a cone shooting from a point P, determines the furthest location from
 * P, at which to place a circle to block the cone.
 *
 * @pre The cone must have nonzero area.
 *
 * @pre \p b must be counterclockwise of \p a.
 *
 * @param a the point such that a vector from p to a represents the right side of the
 * cone.
 *
 * @param b the point such that a vector from p to b represents the left side of the cone.
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
Vector calcBlockOtherRay(const Point &a, const Point &c, const Point &g);

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
bool pointInFrontVector(Point offset, Vector dir, Point p);

/**
 * Returns the circle's tangent points.
 *
 * Returns the points on the circle that form tangent lines with the start point
 */
std::pair<Point, Point> getCircleTangentPoints(const Point &start, const Circle &circle,
                                               double buffer = 0.0);

/**
 * Calculates the pair of Rays that intercept the Circle tangentially with origin at the
 * reference Point
 *
 * @param reference: The point which the tangent vectors will intersect
 * @param circle: The circle to calculate the tangent vectors of
 * @return the mean point of points
 */
std::pair<Ray, Ray> getCircleTangentRaysWithReferenceOrigin(const Point reference,
                                                            const Circle circle);

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
 * Function merges two parameter segments into a single segment ONLY IF one of the
 * segments is contained ENTIRELY within the other
 *
 * @param segment1 : first segment
 * @param segment2 : second segment
 * @return Segment : The longer segment ONLY IF one of the segments is contained entirely
 * within the other
 * @return nullopt : One of the segments is not fully contained in the other, OR the
 * segments are not collinear
 */
std::optional<Segment> mergeFullyOverlappingSegments(Segment segment1, Segment segment2);

/**
 * Function projects Circle objects onto a Segment with respect to an origin Point
 *
 *               projected Segment
 * *______X---------------------------X___________________*  <-- reference Segment
 *          .                        .
 *           .                      .
 *            .                    .
 *             .                  .
 *              .                .
 *               .              .
 *                .   Circle   .
 *                 .  /----\  .
 *                  . |    | .
 *                   .\----/.
 *                    .    .
 *                     .  .
 *                      ..
 *                       X
 *                 Reference Origin
 *
 * @param segment : The reference Segment to project onto
 * @param circles : The vector of circles to project onto the reference Segment
 * @param origin : The Point origin to calculate the projection of the Circles from
 *
 * @return vector<Segment> : All of the projections of the Circles onto the reference
 * Segment
 */
std::vector<Segment> projectCirclesOntoSegment(Segment segment,
                                               std::vector<Circle> circles, Point origin);

/**
 * Function that returns a Segment of the 'Empty' space between the Vector of Segments AND
 * within the parent_segment.
 *
 * NOTE: All segments must be COLLINEAR and must NOT occupy any of the same space
 *       Additionally, the parent segment completely contain ALL segments in the same
 * space
 *
 *  *-------------------------------------------------* <-- Parent segment
 *        *xxxxxxxx*          *xxxxxxxxxx*    *xxxx*    <-- Occupied space (input
 * Segments)
 *  *_____*         *_________*          *____*    *__* <-- Empty space (output Segments)
 *
 * @param segments : The vector of Segments that represent the 'occupied' space. These
 * will be used to find the Segments representing the 'Empty' space
 * @param parent_segment : The Segment representing the entire linear space to calculate
 * the 'Empty' space Segments from
 *
 * @return vector<Segment> : All of the 'Empty' space Segments enclosed in the parent
 * segment
 */
std::vector<Segment> getEmptySpaceWithinParentSegment(std::vector<Segment> segments,
                                                      Segment parent_segment);


/**
 * Function takes in a Vector of Segments and will re-align the Segment to it's equivalent
 * component in the direction of the input Vector. All of the re-aligned Segments will
 * then be combined so that the returned vector<Segment> contains the minimum about of
 * Segments that covers the same linear space in the direction of the input Vector
 *
 * Ex.
 *  Input:  *--------*       *--------*  *
 *                *------*              /
 *                                     /
 *                                    /
 *                                   *
 *
 *  Output: *------------*   *-----------*
 *
 * @param segments : The vector of Segments to be reduced
 * @param direction : The direction that all input vectors will be projected onto
 *
 * @return std::vector<Segment>: The vector of the fewest
 * independent Segments
 */
std::vector<Segment> combineToParallelSegments(std::vector<Segment> segments,
                                               Vector direction);

/**
 * Returns the binary trespass score of a point and rectangle
 *
 * @param point The point to check for trespassing
 * @param rectangle The rectangle to check for trespassing by the Point parameter
 * @return 1 if the point exists within the rectangle, or on the boundry of the rectangle
 *         0 if the point exists outside of the rectangle
 */
int calcBinaryTrespassScore(const Rectangle &rectangle, const Point &point);

/**
 * Finds all circles which do not contain a point in them within the given rectangle.
 *
 * NOTE: this only guarantees that the center of each circle is within the
 *       rectangle, some portion of the circle may extend outside the rectangle
 *
 * @param bounding_box The rectangle in which to look for open circles
 * @param points The points that must not lie within the circles
 *
 * @return A list of circles, sorted in descending order of radius. If no points were
 * provided, returns an empty list. Any points outside the bounding_box are ommitted.
 */
std::vector<Circle> findOpenCircles(Rectangle bounding_box, std::vector<Point> points);

Polygon circleToPolygon(const Circle &circle, size_t num_points);

/**
 *
 * Finds the point in the testPoints vector that is closest to the originPoint.
 *
 * @param originPoint
 * @param testPoints
 * @return The point in testPoints closest to testPoints.
 */
std::optional<Point> findClosestPoint(const Point &origin_point,
                                      std::vector<Point> test_points);

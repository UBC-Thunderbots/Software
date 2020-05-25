#pragma once

#include <cstddef>
#include <optional>
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

double length(const Segment &segment);

double lengthSquared(const Segment &segment);

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
 * returns perpendicular offset from line x0-x1 to point p
 */
double offsetToLine(Point x0, Point x1, Point p);

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

/**
 * Returns the mean of a list of points
 *
 * @param points the vector of points
 * @return the mean point of points
 */
Point getPointsMean(const std::vector<Point> &points);

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

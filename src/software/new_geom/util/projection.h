#pragma once

#include <algorithm>
#include <optional>
#include <vector>

#include "software/new_geom/circle.h"
#include "software/new_geom/point.h"
#include "software/new_geom/ray.h"
#include "software/new_geom/segment.h"
#include "software/new_geom/util/collinear.h"
#include "software/new_geom/util/contains.h"
#include "software/new_geom/util/intersection.h"
#include "software/new_geom/vector.h"

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
std::vector<Segment> projectSegmentsOntoVector(std::vector<Segment> segments,
                                               Vector direction);

#pragma once

#include <optional>
#include <vector>

#include "software/new_geom/segment.h"
#include "software/new_geom/vector.h"

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
std::vector<Segment> realignSegmentsOntoVector(std::vector<Segment> segments,
                                               Vector direction);

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

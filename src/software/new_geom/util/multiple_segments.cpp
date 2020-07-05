#include "software/new_geom/util/multiple_segments.h"

#include <algorithm>

#include "software/new_geom/util/collinear.h"
#include "software/new_geom/util/contains.h"

std::vector<Segment> realignSegmentsOntoVector(std::vector<Segment> segments,
                                               Vector direction)
{
    std::vector<Segment> projected_segments = {};


    // Project all Segments onto the direction Vector
    for (Segment segment : segments)
    {
        // The projection of the Segment without including the original Segment location
        Vector raw_projection = segment.toVector().project(direction);

        // Only count projections that have a non-zero magnitude
        if (raw_projection.lengthSquared() > FIXED_EPSILON)
        {
            projected_segments.push_back(
                Segment(segment.getSegStart(), segment.getSegStart() + raw_projection));
        }
    }
    std::vector<Segment> unique_segments;

    unsigned int j = 0;
    // Loop through all segments and combine segments
    // to reduce the vector to the smallest number of independent (not overlapping)
    // segments
    while (projected_segments.size() > 0)
    {
        std::optional<Segment> temp_segment;
        unique_segments.push_back(projected_segments[0]);
        projected_segments.erase(projected_segments.begin());

        for (unsigned int i = 0; i < projected_segments.size(); i++)
        {
            temp_segment = mergeOverlappingParallelSegments(unique_segments[j],
                                                            projected_segments[i]);

            if (temp_segment.has_value())
            {
                unique_segments[j] = temp_segment.value();
                // Remove segments[i] from the list as it is not unique
                projected_segments.erase(projected_segments.begin() + i);
                i--;
            }
        }
        j++;
    }

    return unique_segments;
}

std::optional<Segment> mergeOverlappingParallelSegments(Segment segment1,
                                                        Segment segment2)
{
    std::optional<Segment> redundant_segment =
        mergeFullyOverlappingSegments(segment1, segment2);

    // If the segments are not parallel, then return std::nullopt. (The segments are
    // parallel of all points are collinear)
    if (!collinear(segment1, segment2))
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
        // segment2,getEnd() and the point furthest from segment2.getEnd()
        return (segment1.getSegStart() - segment2.getEnd()).lengthSquared() >
                       (segment1.getEnd() - segment2.getEnd()).lengthSquared()
                   ? Segment(segment1.getSegStart(), segment2.getEnd())
                   : Segment(segment1.getEnd(), segment2.getEnd());
    }
    // Now check if the end of segment2 lays inside segment1
    else if (contains(segment1, segment2.getEnd()))
    {
        // If segment2.getSegStart() lays in segment1, then the combined segment is
        // segment2,getEnd() and the point furtherst from segmen2.getEnd()
        return (segment1.getSegStart() - segment2.getSegStart()).lengthSquared() >
                       (segment1.getEnd() - segment2.getSegStart()).lengthSquared()
                   ? Segment(segment1.getSegStart(), segment2.getSegStart())
                   : Segment(segment1.getEnd(), segment2.getSegStart());
    }
    return std::nullopt;
}

std::optional<Segment> mergeFullyOverlappingSegments(Segment segment1, Segment segment2)
{
    // If the segments are not parallel, then return std::nullopt. (The segments are
    // parallel if all points are collinear)
    if (!collinear(segment1, segment2))
    {
        return std::nullopt;
    }

    Segment largest_segment, smallest_segment;
    // Grab the largest segment
    if (segment1.toVector().lengthSquared() > segment2.toVector().lengthSquared())
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

std::vector<Segment> getEmptySpaceWithinParentSegment(std::vector<Segment> segments,
                                                      Segment parent_segment)
{
    // Make sure the starting point of all segments is closer to the start of the
    // reference segment to simplify the evaluation
    for (auto &unordered_seg : segments)
    {
        if ((parent_segment.getSegStart() - unordered_seg.getSegStart()).length() >
            (parent_segment.getSegStart() - unordered_seg.getEnd()).length())
        {
            // We need to flip the start/end of the segment
            Segment temp = unordered_seg;
            unordered_seg.setSegStart(temp.getEnd());
            unordered_seg.setEnd(temp.getSegStart());
        }
    }

    // Now we must sort the segments so that we can iterate through them in order to
    // generate open angles sort using a lambda expression
    // We sort the segments based on how close their 'start' point is to the 'start'
    // of the reference Segment
    std::sort(segments.begin(), segments.end(), [parent_segment](Segment &a, Segment &b) {
        return (parent_segment.getSegStart() - a.getSegStart()).length() <
               (parent_segment.getSegStart() - b.getSegStart()).length();
    });

    // Now we need to find the largest open segment/angle
    std::vector<Segment> open_segs;

    // The first Angle is between the reference Segment and the first obstacle Segment
    // After this one, ever open angle is between segment(i).end and
    // segment(i+1).start
    open_segs.push_back(
        Segment(parent_segment.getSegStart(), segments.front().getSegStart()));

    // The 'open' Segment in the space between consecutive 'blocking' Segments
    for (std::vector<Segment>::const_iterator it = segments.begin();
         it != segments.end() - 1; it++)
    {
        open_segs.push_back(Segment(it->getEnd(), (it + 1)->getSegStart()));
    }

    // Lastly, the final open angle is between obstacles.end().getEnd() and
    // reference_segment.getEnd()
    open_segs.push_back(Segment(segments.back().getEnd(), parent_segment.getEnd()));

    // Remove all zero length open Segments
    for (std::vector<Segment>::const_iterator it = open_segs.begin();
         it != open_segs.end();)
    {
        // 2 * FIXED_EPSILON for error needed:
        // - segment subtraction (end - start)
        // - mathcalls `hypot` from `length` function call
        if (it->length() < 2 * FIXED_EPSILON)
        {
            open_segs.erase(it);
        }
        else
        {
            it++;
        }
    }

    return open_segs;
}

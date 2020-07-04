#include "software/new_geom/util/multiple_segments.h"

#include <gtest/gtest.h>

// Test that function returns the larger segment when considering 2 redundant segments
TEST(MultipleSegmentsTest, test_segment_redundancy_segments_are_redundant)
{
    Segment segment1 = Segment(Point(-1, -1), Point(1, 1));

    Segment segment2 = Segment(Point(-2, -2), Point(2, 2));

    std::optional<Segment> largest_segment =
        mergeFullyOverlappingSegments(segment1, segment2);

    EXPECT_EQ(largest_segment.value(), segment2);
}

// Test that function returns the larger segment when considering 2 non-parallel segments
TEST(MultipleSegmentsTest, test_segment_redundancy_segments_are_not_parallel)
{
    Segment segment1 = Segment(Point(2, 2), Point(1, -2));

    Segment segment2 = Segment(Point(1, 4), Point(1, -4));

    std::optional<Segment> largest_segment =
        mergeFullyOverlappingSegments(segment1, segment2);

    EXPECT_EQ(largest_segment, std::nullopt);
}

// Test that function returns one of the segments if they are exactly the same
TEST(MultipleSegmentsTest, test_segment_redundancy_segments_are_the_same)
{
    Segment segment1 = Segment(Point(2, 2), Point(1, -2));

    Segment segment2 = Segment(Point(2, 2), Point(1, -2));

    std::optional<Segment> largest_segment =
        mergeFullyOverlappingSegments(segment1, segment2);

    EXPECT_EQ(largest_segment.value(), segment1);
}

// Test if segments are merged if they are parallel and only partially overlapping
TEST(MultipleSegmentsTest, test_merge_segment_partially_overlapping)
{
    Segment segment1 = Segment(Point(-2, -2), Point(2, 2));

    Segment segment2 = Segment(Point(-1, -1), Point(5, 5));

    std::optional<Segment> merged_segment =
        mergeOverlappingParallelSegments(segment1, segment2);
    EXPECT_EQ(merged_segment.value(), Segment(Point(-2, -2), Point(5, 5)));
}

// Test if segments are merged if they are parallel and only partially overlapping
TEST(MultipleSegmentsTest, test_merge_segment_redundant_segments)
{
    Segment segment1 = Segment(Point(-2, -2), Point(2, 2));

    Segment segment2 = Segment(Point(-1, -1), Point(1, 1));

    std::optional<Segment> merged_segment =
        mergeOverlappingParallelSegments(segment1, segment2);
    EXPECT_EQ(merged_segment.value(), segment1);
}

TEST(MultipleSegmentsTest, test_get_empty_space_between_segment_no_space)
{
    Segment reference = Segment(Point(-10, 0), Point(10, 0));

    Segment seg1 = Segment(Point(-10, 0), Point(0, 0));
    Segment seg2 = Segment(Point(0, 0), Point(10, 0));

    std::vector<Segment> segs = {seg1, seg2};

    std::vector<Segment> open_segs = getEmptySpaceWithinParentSegment(segs, reference);

    EXPECT_EQ(open_segs.size(), 0);
}

TEST(MultipleSegmentsTest, test_get_empty_space_between_segment_2_blocks)
{
    Segment reference = Segment(Point(-10, 0), Point(10, 0));

    Segment seg1 = Segment(Point(-8, 0), Point(0, 0));
    Segment seg2 = Segment(Point(0, 0), Point(8, 0));

    std::vector<Segment> segs = {seg1, seg2};

    std::vector<Segment> open_segs = getEmptySpaceWithinParentSegment(segs, reference);

    EXPECT_EQ(open_segs.size(), 2);

    EXPECT_EQ(open_segs[0].length(), 2);
    EXPECT_EQ(open_segs[1].length(), 2);
}

TEST(MultipleSegmentsTest, test_get_empty_space_between_segment_2_blocks_3_open)
{
    Segment reference = Segment(Point(-10, 0), Point(10, 0));

    Segment seg1 = Segment(Point(-8, 0), Point(-2, 0));
    Segment seg2 = Segment(Point(2, 0), Point(8, 0));

    std::vector<Segment> segs = {seg1, seg2};

    std::vector<Segment> open_segs = getEmptySpaceWithinParentSegment(segs, reference);

    EXPECT_EQ(open_segs.size(), 3);

    EXPECT_EQ(open_segs[0].length(), 2);
    EXPECT_EQ(open_segs[1].length(), 4);
    EXPECT_EQ(open_segs[2].length(), 2);
}

TEST(MultipleSegmentsTest, test_reduce_segments_collinear)
{
    Segment seg1 = Segment(Point(0, 1), Point(0, 2));
    Segment seg2 = Segment(Point(0, 1.5), Point(0, 2.5));
    Segment seg3 = Segment(Point(0, 3), Point(0, 4));
    Segment seg4 = Segment(Point(0, 1.2), Point(0, 1.5));

    Segment seg5 = Segment(Point(0, 6), Point(0, 9));
    Segment seg6 = Segment(Point(0, 6), Point(0, 12));

    Segment seg7 = Segment(Point(0, -2), Point(0, -5));

    std::vector<Segment> segs = {seg1, seg2, seg3, seg4, seg5, seg6, seg7};

    std::optional<std::vector<Segment>> reduced_segs =
        realignSegmentsOntoVector(segs, segs.front().toVector());

    EXPECT_EQ(Segment(Point(0, 1), Point(0, 2.5)), reduced_segs.value()[0]);
}

TEST(MultipleSegmentsTest, test_reduce_segments_perpendicular)
{
    Segment seg1 = Segment(Point(0, 0), Point(0, 10));
    Segment seg2 = Segment(Point(0, 1), Point(1, 1));
    Segment seg3 = Segment(Point(-5, -5), Point(5, -5));


    std::vector<Segment> segs = {seg1, seg2, seg3};

    std::optional<std::vector<Segment>> reduced_segs =
        realignSegmentsOntoVector(segs, segs.front().toVector());

    EXPECT_EQ(reduced_segs->size(), 1);
    EXPECT_EQ(reduced_segs->front().length(), 10);
}


TEST(MultipleSegmentsTest, test_reduce_segments_perpendicular_and_parallel)
{
    Segment seg1 = Segment(Point(0, 0), Point(0, 10));
    Segment seg2 = Segment(Point(0, 1), Point(1, 1));
    Segment seg3 = Segment(Point(-5, -5), Point(5, -5));
    Segment seg4 = Segment(Point(0, 2), Point(0, 15));
    Segment seg5 = Segment(Point(0, 7), Point(9, 7));


    std::vector<Segment> segs = {seg1, seg2, seg3, seg4, seg5};

    std::optional<std::vector<Segment>> reduced_segs =
        realignSegmentsOntoVector(segs, seg1.toVector());

    EXPECT_EQ(reduced_segs->size(), 1);
    EXPECT_EQ(reduced_segs->front().length(), 15);
    EXPECT_EQ(reduced_segs->front().getEnd().y(), 15);
}

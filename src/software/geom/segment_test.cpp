#include "software/geom/segment.h"

#include <gtest/gtest.h>

TEST(SegmentConstructorTests, segment_default_constructor)
{
    Segment s = Segment(Point(0, 0), Point(2, 5));
    EXPECT_EQ(s.getStart(), Point(0, 0));
    EXPECT_EQ(s.getEnd(), Point(2, 5));
}

TEST(SegmentSetTests, segment_set_start)
{
    Segment s = Segment(Point(0, 0), Point(4, 5));
    s.setStart(Point(1, 1));
    EXPECT_EQ(s.getStart(), Point(1, 1));
}

TEST(SegmentSetTests, segment_set_end)
{
    Segment s = Segment(Point(4, 2), Point(5, 7));
    s.setEnd(Point(3, 1));
    EXPECT_EQ(s.getEnd(), Point(3, 1));
}

TEST(SegmentLengthTests, segment_get_length_one_dimension)
{
    Segment s = Segment(Point(1, 2), Point(1, 5));
    EXPECT_EQ(s.length(), 3);
}

TEST(SegmentLengthTests, segment_get_length_two_dimension)
{
    Segment s = Segment(Point(3, 5), Point(6, 10));
    EXPECT_EQ(s.length(), sqrt(pow(3, 2) + pow(5, 2)));
}

TEST(SegmentLengthTests, segment_get_lengthsquared)
{
    Segment s = Segment(Point(1, 2), Point(4, 6));
    EXPECT_EQ(s.lengthSquared(), 25);
}

TEST(SegmentReverseTests, segment_reverse)
{
    Segment s        = Segment(Point(1, 2), Point(3, 4));
    Segment expected = Segment(Point(3, 4), Point(1, 2));
    EXPECT_EQ(s.reverse(), expected);
}

TEST(SegmentVectorTests, segment_tovector)
{
    Segment s       = Segment(Point(1, 2), Point(3, 4));
    Vector expected = Vector(2, 2);
    EXPECT_EQ(s.toVector(), expected);
}

TEST(SegmentOperatorTests, segment_operator_equal)
{
    Segment s1 = Segment(Point(3, 5), Point(8, 6));
    Segment s2 = Segment(Point(3, 5), Point(8, 6));
    EXPECT_TRUE(s1.operator==(s2));
}

TEST(SegmentOperatorTests, segment_operator_unequal_start)
{
    Segment s1 = Segment(Point(7, 6), Point(2, 3));
    Segment s2 = Segment(Point(2, 5), Point(2, 3));
    EXPECT_FALSE(s1.operator==(s2));
}

TEST(SegmentOperatorTests, segment_operator_unequal_end)
{
    Segment s1 = Segment(Point(7, 6), Point(2, 3));
    Segment s2 = Segment(Point(7, 6), Point(1, 3));
    EXPECT_FALSE(s1.operator==(s2));
}

TEST(SegmentOperatorTests, segment_operator_unequal_start_end)
{
    Segment s1 = Segment(Point(3, 6), Point(2, 3));
    Segment s2 = Segment(Point(7, 6), Point(1, 3));
    EXPECT_FALSE(s1.operator==(s2));
}

TEST(SegmentOffsetTests, segment_offset_zero)
{
    Segment s = Segment(Point(3, 5), Point(8, 6));
    Vector v = Vector(0, 0);
    Segment add = s+v;
    EXPECT_DOUBLE_EQ(add.getStart().x(), 3);
    EXPECT_DOUBLE_EQ(add.getStart().y(), 5);
    EXPECT_DOUBLE_EQ(add.getEnd().x(), 8);
    EXPECT_DOUBLE_EQ(add.getEnd().y(), 6);
}

TEST(SegmentOffsetTests, segment_offset_diag)
{
    Segment s = Segment(Point(3, 5), Point(8, 6));
    Vector v = Vector(1, 2);
    Segment add = s+v;
    EXPECT_DOUBLE_EQ(add.getStart().x(), 4);
    EXPECT_DOUBLE_EQ(add.getStart().y(), 7);
    EXPECT_DOUBLE_EQ(add.getEnd().x(), 9);
    EXPECT_DOUBLE_EQ(add.getEnd().y(), 8);
}

TEST(SegmentOffsetTests, segment_offset_negative)
{
    Segment s = Segment(Point(3, 5), Point(8, 6));
    Vector v = Vector(1, -2);
    Segment add = v+s;
    EXPECT_DOUBLE_EQ(add.getStart().x(), 4);
    EXPECT_DOUBLE_EQ(add.getStart().y(), 3);
    EXPECT_DOUBLE_EQ(add.getEnd().x(), 9);
    EXPECT_DOUBLE_EQ(add.getEnd().y(), 4);
}

TEST(SegmentOffsetTests, segment_backset_zero)
{
    Segment s = Segment(Point(3, 5), Point(8, 6));
    Vector v = Vector(0, 0);
    Segment add = s-v;
    EXPECT_DOUBLE_EQ(add.getStart().x(), 3);
    EXPECT_DOUBLE_EQ(add.getStart().y(), 5);
    EXPECT_DOUBLE_EQ(add.getEnd().x(), 8);
    EXPECT_DOUBLE_EQ(add.getEnd().y(), 6);
}

TEST(SegmentOffsetTests, segment_backset_diag)
{
    Segment s = Segment(Point(3, 5), Point(8, 6));
    Vector v = Vector(1, 2);
    Segment add = s-v;
    EXPECT_DOUBLE_EQ(add.getStart().x(), 2);
    EXPECT_DOUBLE_EQ(add.getStart().y(), 3);
    EXPECT_DOUBLE_EQ(add.getEnd().x(), 7);
    EXPECT_DOUBLE_EQ(add.getEnd().y(), 4);
}

TEST(SegmentOffsetTests, segment_backset_negative)
{
    Segment s = Segment(Point(3, 5), Point(8, 6));
    Vector v = Vector(1, -2);
    Segment add = s-v;
    EXPECT_DOUBLE_EQ(add.getStart().x(), 2);
    EXPECT_DOUBLE_EQ(add.getStart().y(), 7);
    EXPECT_DOUBLE_EQ(add.getEnd().x(), 7);
    EXPECT_DOUBLE_EQ(add.getEnd().y(), 8);
}

TEST(SegmentMidPointTest, test_mid_point_1)
{
    Segment segment(Point(202, 15), Point(202, -15));
    EXPECT_EQ(Point(202, 0), segment.midPoint());
}

TEST(SegmentMidPointTest, test_mid_point_2)
{
    Segment segment(Point(-2, 4), Point(5, 3));
    EXPECT_EQ(Point(1.5, 3.5), segment.midPoint());
}

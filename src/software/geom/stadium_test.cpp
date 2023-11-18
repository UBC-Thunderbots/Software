#include "software/geom/stadium.h"

#include <gtest/gtest.h>

TEST(StadiumConstructorTests, stadium_default_constructor)
{
    Stadium s = Stadium();
    EXPECT_EQ(0, s.radius());
    EXPECT_EQ(Segment(), s.length());
}

TEST(StadiumConstructorTests, stadium_segment_constructor)
{
    Stadium s = Stadium(Segment(Point(1, 2), Point(-1, 3)), 2);
    EXPECT_EQ(2, s.radius());
    EXPECT_EQ(Segment(Point(1,2), Point(-1,3)), s.length());
}

TEST(StadiumConstructorTests, stadium_points_constructor)
{
    Stadium s = Stadium(Point(1, 2), Point(-1, 3), 4);
    EXPECT_EQ(4, s.radius());
    EXPECT_EQ(Segment(Point(1,2), Point(-1,3)), s.length());
}

TEST(StadiumConstructorTests, stadium_point_vector_constructor)
{
    Stadium s = Stadium(Point(1, 2), Vector(-2, 1), 3);
    EXPECT_EQ(3, s.radius());
    EXPECT_EQ(Segment(Point(1,2), Point(-1,3)), s.length());
}

TEST(StadiumConstructorTests, stadium_point_circle_constructor)
{
    Stadium s = Stadium(Point(1, 2), Circle(Point(-8, 4), 1));
    EXPECT_EQ(1, s.radius());
    EXPECT_EQ(Segment(Point(1,2), Point(-8,4)), s.length());
}

TEST(StadiumConstructorTests, stadium_circle_vector_constructor)
{
    Stadium s = Stadium(Circle(Point(-2, 1), 6), Vector(1, 2));
    EXPECT_EQ(6, s.radius());
    EXPECT_EQ(Segment(Point(-2,1), Point(-1,3)), s.length());
}

TEST(StadiumConstructorTests, stadium_invalid_points_constructor)
{
    EXPECT_THROW(Stadium(Point(1, 2), Point(-1, 3), -2), std::invalid_argument);
}

TEST(StadiumConstructorTests, stadium_invalid_segment_constructor)
{
    EXPECT_THROW(Stadium(Segment(Point(1, 2), Point(-1, 3)), -4), std::invalid_argument);
}


TEST(StadiumAreaTests, stadium_default_area_test)
{
    Stadium s = Stadium();
    EXPECT_EQ(s.area(), 0);
}

TEST(StadiumAreaTests, stadium_flat_area_test)
{
    Stadium s = Stadium(Segment(Point(-1, 0), Point(1, 0)), 1);
    EXPECT_DOUBLE_EQ(s.area(), M_PI + 4);
}
TEST(StadiumAreaTests, stadium_flat_area_test_bigger)
{
    Stadium s = Stadium(Segment(Point(-1, 0), Point(1, 0)), 2);
    EXPECT_DOUBLE_EQ(s.area(), M_PI * 4 + 8);
}
TEST(StadiumAreaTests, stadium_angled_area_test)
{
    Stadium s = Stadium(Segment(Point(0, 0), Point(2*cos(45), 2*sin(45))), 1);
    EXPECT_DOUBLE_EQ(s.area(), M_PI + 4);
}
TEST(StadiumAreaTests, stadium_vertical_area_test)
{
    Stadium s = Stadium(Segment(Point(0, -1), Point(0,1)), 1);
    EXPECT_DOUBLE_EQ(s.area(), M_PI + 4);
}

TEST(StadiumSpanTests, stadium_flat_span_test)
{
    Stadium s = Stadium(Segment(Point(-1, 0), Point(1, 0)), 1);
    EXPECT_DOUBLE_EQ(s.x_span(), 4);
    EXPECT_DOUBLE_EQ(s.y_span(), 2);
}

TEST(StadiumSpanTests, stadium_flat_span_test_reversed)
{
    Stadium s = Stadium(Segment(Point(1, 0), Point(-1, 0)), 1);
    EXPECT_DOUBLE_EQ(s.x_span(), 4);
    EXPECT_DOUBLE_EQ(s.y_span(), 2);
}

TEST(StadiumSpanTests, stadium_flat_span_test_bigger)
{
    Stadium s = Stadium(Segment(Point(-1, 0), Point(1, 0)), 2);
    EXPECT_DOUBLE_EQ(s.x_span(), 6);
    EXPECT_DOUBLE_EQ(s.y_span(), 4);
}

TEST(StadiumSpanTests, stadium_angled_span_test)
{
    Stadium s = Stadium(Segment(Point(-2, 1), Point(0, 4)), 1);
    EXPECT_DOUBLE_EQ(s.x_span(), 4);
    EXPECT_DOUBLE_EQ(s.y_span(), 5);
}

TEST(StadiumSpanTests, stadium_vertical_span_test)
{
    Stadium s = Stadium(Segment(Point(0, 1), Point(0, -1)), 1);
    EXPECT_DOUBLE_EQ(s.x_span(), 2);
    EXPECT_DOUBLE_EQ(s.y_span(), 4);
}


TEST(StadiumEqualsTests, stadium_default_equals_default_test)
{
    Stadium s1 = Stadium();
    Stadium s2 = Stadium();

    EXPECT_TRUE(s1 == s2);
}

TEST(StadiumEqualsTests, stadium_segment_equals_default_test)
{
    Stadium s1 = Stadium(Segment(Point(0,0), Point(0,0)), 0);
    Stadium s2 = Stadium();

    EXPECT_TRUE(s1 == s2);
}

TEST(StadiumEqualsTests, stadium_segment_equals_point_test)
{
    Stadium s1 = Stadium(Segment(Point(1,2), Point(2,-2)), 3);
    Stadium s2 = Stadium(Point(1,2), Point(2,-2), 3);

    EXPECT_TRUE(s1 == s2);
}

TEST(StadiumEqualsTests, stadium_point_equals_point_test)
{
    Stadium s1 = Stadium(Point(2,2), Point(4,-2), 3);
    Stadium s2 = Stadium(Point(2,2), Point(4,-2), 3);

    EXPECT_TRUE(s1 == s2);
}

TEST(StadiumEqualsTests, stadium_segment_equals_segment_test)
{
    Stadium s1 = Stadium(Segment(Point(1,-2), Point(2,1)), 6);
    Stadium s2 = Stadium(Segment(Point(1,-2), Point(2,1)), 6);

    EXPECT_TRUE(s1 == s2);
}

TEST(StadiumEqualsTests, stadium_reverse_equals_normal_test)
{
    Stadium s1 = Stadium(Segment(Point(1,-2), Point(2,1)), 2);
    Stadium s2 = Stadium(Segment(Point(2,1), Point(1,-2)), 2);

    EXPECT_TRUE(s1 == s2);
}

TEST(StadiumNotEqualsTests, stadium_segment_not_equals_default_test)
{
    Stadium s1 = Stadium(Segment(Point(1,0), Point(0,0)), 0);
    Stadium s2 = Stadium();

    EXPECT_TRUE(s1 != s2);
}

TEST(StadiumNotEqualsTests, stadium_segment_not_equals_point_test)
{
    Stadium s1 = Stadium(Segment(Point(1,2), Point(2,-2)), 3);
    Stadium s2 = Stadium(Point(1,2), Point(2,-3), 3);

    EXPECT_TRUE(s1 != s2);
}

TEST(StadiumNotEqualsTests, stadium_point_not_equals_point_test)
{
    Stadium s1 = Stadium(Point(2,1), Point(5,-1), 3);
    Stadium s2 = Stadium(Point(2,2), Point(4,-2), 3);

    EXPECT_TRUE(s1 != s2);
}

TEST(StadiumNotEqualsTests, stadium_segment_not_equals_segment_test)
{
    Stadium s1 = Stadium(Segment(Point(1,-2), Point(1,1)), 6);
    Stadium s2 = Stadium(Segment(Point(1,-3), Point(2,1)), 6);

    EXPECT_TRUE(s1 != s2);
}

TEST(StadiumNotEqualsTests, stadium_radius_not_equals_test)
{
    Stadium s1 = Stadium(Segment(Point(1,-2), Point(2,1)), 3);
    Stadium s2 = Stadium(Segment(Point(1,-2), Point(2,1)), 1);

    EXPECT_TRUE(s1 != s2);
}

TEST(StadiumNotEqualsTests, stadium_radius_not_equals_reverse_test)
{
    Stadium s1 = Stadium(Segment(Point(1,-2), Point(2,1)), 3);
    Stadium s2 = Stadium(Segment(Point(2,1), Point(1,-2)), 2);

    EXPECT_TRUE(s1 != s2);
}


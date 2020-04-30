#include "software/new_geom/util/contains.h"

#include <gtest/gtest.h>

TEST(ContainsTest, segment_in_circle)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({-2, 2}, {4, 0});
    EXPECT_TRUE(containsNew(c, s));
}

TEST(ContainsTest, segment_one_point_in_circle_other_point_on_circle_edge)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({-2, 2}, {5, 1});
    EXPECT_TRUE(containsNew(c, s));
}

TEST(ContainsTest, segment_one_point_in_circle_other_point_out)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({-2, 2}, {20, -10});
    EXPECT_FALSE(containsNew(c, s));
}


TEST(ContainsTest, segment_both_points_on_circle_edge)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({1, 5}, {5, 1});
    EXPECT_TRUE(containsNew(c, s));
}

TEST(ContainsTest,
     segment_one_point_on_circle_edge_other_point_not_passing_through_circle)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({1, 5}, {6, 6});
    EXPECT_FALSE(containsNew(c, s));
}

TEST(ContainsTest, segment_both_points_out_of_circle_passing_through)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({-4, 4}, {2, -4});
    EXPECT_FALSE(containsNew(c, s));
}

TEST(ContainsTest, segment_completely_out_of_circle)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({-50, -50}, {30, -30});
    EXPECT_FALSE(containsNew(c, s));
}

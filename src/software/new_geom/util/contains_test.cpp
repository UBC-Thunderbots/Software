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
}

TEST(ContainsTest, segment_one_point_in_circle_other_point_out)
{
}


TEST(ContainsTest, segment_both_points_on_circle_edge)
{
}

TEST(ContainsTest, segment_one_point_on_circle_edge_other_point_out)
{
}

TEST(ContainsTest, segment_out_of_circle)
{
}

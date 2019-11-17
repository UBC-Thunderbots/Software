#include "software/new_geom/line.h"

#include <gtest/gtest.h>

TEST(LineTest, default_constructor)
{
    Line l;
    EXPECT_DOUBLE_EQ(l.getSlope(), 0);
    EXPECT_DOUBLE_EQ(l.getYIntercept(), 0);

    for (size_t i = 0; i < 10; i++)
    {
        EXPECT_DOUBLE_EQ(l.valueAt(i), 0);
    }
}

TEST(LineTest, intercept_slope_constructor)
{
    Line l(5.2, 6.3);
    EXPECT_DOUBLE_EQ(l.getYIntercept(), 5.2);
    EXPECT_DOUBLE_EQ(l.getSlope(), 6.3);

    EXPECT_DOUBLE_EQ(l.valueAt(-1.2), -2.36);
    EXPECT_DOUBLE_EQ(l.valueAt(0), 5.2);
    EXPECT_DOUBLE_EQ(l.valueAt(4), 30.4);
    EXPECT_DOUBLE_EQ(l.valueAt(410), 2588.2);
}

TEST(LineTest, test_set_coeff)
{
    Line l(5.2, 6.3);
    l.setYIntercept(3.7);
    l.setSlope(-2.1);
    EXPECT_DOUBLE_EQ(l.getYIntercept(), 3.7);
    EXPECT_DOUBLE_EQ(l.getSlope(), -2.1);

    EXPECT_DOUBLE_EQ(l.valueAt(-4.2), 12.52);
    EXPECT_DOUBLE_EQ(l.valueAt(0), 3.7);
    EXPECT_DOUBLE_EQ(l.valueAt(8.2), -13.52);
}

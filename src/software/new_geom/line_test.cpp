#include "software/new_geom/line.h"

#include <gtest/gtest.h>

TEST(LineTest, default_constructor)
{
    Line l;
    for (size_t i = 0; i < 10; i++)
    {
        EXPECT_DOUBLE_EQ(l.getCoeff(i), 0);
        EXPECT_DOUBLE_EQ(l.valueAt(i), 0);
    }
    EXPECT_DOUBLE_EQ(l.getOrder(), 0);
}

TEST(LineTest, intercept_slope_constructor)
{
    Line l(5.2, 6.3);
    EXPECT_DOUBLE_EQ(l.getCoeff(0), 5.2);
    EXPECT_DOUBLE_EQ(l.getCoeff(1), 6.3);
    EXPECT_DOUBLE_EQ(l.getOrder(), 1);

    EXPECT_DOUBLE_EQ(l.valueAt(-1.2), -2.36);
    EXPECT_DOUBLE_EQ(l.valueAt(0), 5.2);
    EXPECT_DOUBLE_EQ(l.valueAt(4), 30.4);
    EXPECT_DOUBLE_EQ(l.valueAt(410), 2588.2);
}

TEST(LineTest, test_set_coeff)
{
    Line l(5.2, 6.3);
    l.setCoeff(0, 3.7);
    l.setCoeff(1, -2.1);
    EXPECT_DOUBLE_EQ(l.getCoeff(0), 3.7);
    EXPECT_DOUBLE_EQ(l.getCoeff(1), -2.1);
    EXPECT_DOUBLE_EQ(l.getOrder(), 1);

    EXPECT_DOUBLE_EQ(l.valueAt(-4.2), 12.52);
    EXPECT_DOUBLE_EQ(l.valueAt(0), 3.7);
    EXPECT_DOUBLE_EQ(l.valueAt(8.2), -13.52);
}

TEST(LineTest, invalid_set_coeff)
{
    Line l(2, 1);
    try
    {
        l.setCoeff(2, 5);
    }
    catch (std::invalid_argument &e)
    {
        SUCCEED();
        return;
    }
    ADD_FAILURE() << "Able to set the coefficient of order 2 term";
}

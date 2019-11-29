#include "software/new_geom/line.h"

#include <gtest/gtest.h>

TEST(LineTest, default_constructor)
{
    Line l;
    EXPECT_EQ(l.getFirst(), Point());
    EXPECT_EQ(l.getSecond(), Point());

    for (size_t i = 0; i < 10; i++)
    {
        EXPECT_EQ(l.valueAt(i), Point());
    }
}

TEST(LineTest, two_points_constructor)
{
    double x1 = -1.0;
    double y1 = 2.0;
    double x2 = 3.0;
    double y2 = 4.0;
    Line l(Point(x1, y1), Point(x2, y2));
    EXPECT_EQ(l.getFirst(), Point(x1, y1));
    EXPECT_EQ(l.getSecond(), Point(x2, y2));
    EXPECT_DOUBLE_EQ(l.getSlope(), 0.5);

    double angle = std::atan2(y2 - y1, x2 - x1);
    EXPECT_EQ(l.valueAt(-20000), Point(x1 + -20000 * std::cos(angle), y1 + -20000 * std::sin(angle)));
    EXPECT_EQ(l.valueAt(-2.5), Point(x1 + -2.5 * std::cos(angle), y1 + -2.5 * std::sin(angle)));
    EXPECT_EQ(l.valueAt(0.0), Point(x1, y1));
    EXPECT_EQ(l.valueAt(2.5), Point(x1 + 2.5 * std::cos(angle), y1 + 2.5 * std::sin(angle)));
    EXPECT_EQ(l.valueAt(20000), Point(x1 + 20000 * std::cos(angle), y1 + 20000 * std::sin(angle)));
}

TEST(LineTest, two_points_constructor_reverse)
{
    double x1 = -1.0;
    double y1 = 2.0;
    double x2 = -5.0;
    double y2 = 0.0;
    Line l(Point(x1, y1), Point(x2, y2));
    EXPECT_EQ(l.getFirst(), Point(x1, y1));
    EXPECT_EQ(l.getSecond(), Point(x2, y2));
    EXPECT_DOUBLE_EQ(l.getSlope(), 0.5);

    double angle = std::atan2(y2 - y1, x2 - x1);
    EXPECT_EQ(l.valueAt(-20000), Point(x1 + -20000 * std::cos(angle), y1 + -20000 * std::sin(angle)));
    EXPECT_EQ(l.valueAt(-2.5), Point(x1 + -2.5 * std::cos(angle), y1 + -2.5 * std::sin(angle)));
    EXPECT_EQ(l.valueAt(0.0), Point(x1, y1));
    EXPECT_EQ(l.valueAt(2.5), Point(x1 + 2.5 * std::cos(angle), y1 + 2.5 * std::sin(angle)));
    EXPECT_EQ(l.valueAt(20000), Point(x1 + 20000 * std::cos(angle), y1 + 20000 * std::sin(angle)));
}

TEST(LineTest, set_first)
{
    double x1 = -3.0;
    double y1 = 4.0;
    double x2 = 8.0;
    double y2 = 2.0;
    Line l(Point(-1.0, 2.0), Point(x2, y2));
    l.setFirst(Point(x1, y1));
    EXPECT_EQ(l.getFirst(), Point(x1, y1));
    EXPECT_DOUBLE_EQ(l.getSlope(), -2.0 / 11.0);

    double angle = std::atan2(y2 - y1, x2 - x1);
    EXPECT_EQ(l.valueAt(-20000), Point(x1 + -20000 * std::cos(angle), y1 + -20000 * std::sin(angle)));
    EXPECT_EQ(l.valueAt(-2.5), Point(x1 + -2.5 * std::cos(angle), y1 + -2.5 * std::sin(angle)));
    EXPECT_EQ(l.valueAt(0.0), Point(x1, y1));
    EXPECT_EQ(l.valueAt(2.5), Point(x1 + 2.5 * std::cos(angle), y1 + 2.5 * std::sin(angle)));
    EXPECT_EQ(l.valueAt(20000), Point(x1 + 20000 * std::cos(angle), y1 + 20000 * std::sin(angle)));
}

TEST(LineTest, set_second)
{
    double x1 = -3.0;
    double y1 = 4.0;
    double x2 = 8.0;
    double y2 = 2.0;
    Line l(Point(x1, y1), Point(3.0, 4.0));
    l.setSecond(Point(x2, y2));
    EXPECT_EQ(l.getSecond(), Point(x2, y2));
    EXPECT_DOUBLE_EQ(l.getSlope(), -2.0 / 11.0);

    double angle = std::atan2(y2 - y1, x2 - x1);
    EXPECT_EQ(l.valueAt(-20000), Point(x1 + -20000 * std::cos(angle), y1 + -20000 * std::sin(angle)));
    EXPECT_EQ(l.valueAt(-2.5), Point(x1 + -2.5 * std::cos(angle), y1 + -2.5 * std::sin(angle)));
    EXPECT_EQ(l.valueAt(0.0), Point(x1, y1));
    EXPECT_EQ(l.valueAt(2.5), Point(x1 + 2.5 * std::cos(angle), y1 + 2.5 * std::sin(angle)));
    EXPECT_EQ(l.valueAt(20000), Point(x1 + 20000 * std::cos(angle), y1 + 20000 * std::sin(angle)));
}

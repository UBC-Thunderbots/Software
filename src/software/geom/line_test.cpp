#include "software/geom/line.h"

#include <gtest/gtest.h>

TEST(LineTest, two_points_constructor)
{
    Line l(Point(-1.0, 2.0), Point(3.0, 4.0));  // -0.5x + y - 2.5 = 0
    EXPECT_EQ(l.toNormalUnitVector(), Vector(-1, 2).normalize());
    Line::Coeffs expectedCoeffs = {-2, 4, -10};
    Line::Coeffs coeffs         = l.getCoeffs();
    EXPECT_DOUBLE_EQ(coeffs.a, expectedCoeffs.a);
    EXPECT_DOUBLE_EQ(coeffs.b, expectedCoeffs.b);
    EXPECT_DOUBLE_EQ(coeffs.c, expectedCoeffs.c);
}

TEST(LineTest, two_points_constructor_invalid)
{
    try
    {
        Line l(Point(1, 1), Point(1, 1));
        FAIL() << "A Line was created with two equal Points";
    }
    catch (std::runtime_error &e)
    {
        EXPECT_EQ(e.what(), std::string("Cannot create a Line with two equal Points"));
    }
    catch (...)
    {
        FAIL() << "Expected std::runtime_error";
    }
}

TEST(LineTest, to_normal_unit_vector)
{
    Line l1(Point(-5, -4.5), Point(0, 3));  // -1.5x + y - 3 = 0
    Line l2(Point(-2, -9), Point(4, 0));    // -1.5x + y + 6 = 0
    EXPECT_EQ(l1.toNormalUnitVector(), l2.toNormalUnitVector());
}

TEST(LineTest, vertical_line)
{
    Line l(Point(1, -1), Point(1, 1));  // x = 1
    EXPECT_EQ(l.toNormalUnitVector(), Vector(-1, 0));
    Line::Coeffs expectedCoeffs = {-2, 0, 2};
    Line::Coeffs coeffs         = l.getCoeffs();
    EXPECT_DOUBLE_EQ(coeffs.a, expectedCoeffs.a);
    EXPECT_DOUBLE_EQ(coeffs.b, expectedCoeffs.b);
    EXPECT_DOUBLE_EQ(coeffs.c, expectedCoeffs.c);
}

TEST(LineTest, swapXY)
{
    Line l(Point(-1.0, 2.0), Point(3.0, 4.0));  // -0.5x + y - 2.5 = 0
    l.swapXY();
    EXPECT_EQ(l.toNormalUnitVector(), Vector(2, -1).normalize());
    Line::Coeffs expectedCoeffs = {4, -2, -10};
    Line::Coeffs coeffs         = l.getCoeffs();
    EXPECT_DOUBLE_EQ(coeffs.a, expectedCoeffs.a);
    EXPECT_DOUBLE_EQ(coeffs.b, expectedCoeffs.b);
    EXPECT_DOUBLE_EQ(coeffs.c, expectedCoeffs.c);
}

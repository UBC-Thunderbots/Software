#include "geom/point.h"

#include <gtest/gtest.h>

TEST(AngleTest, accesors)
{
    EXPECT_DOUBLE_EQ(0, Point().x());
}

TEST(PlusEqualOperatorTest, plus_equal_operator_test)
{
    Point p = Point(1, 2);
    Point q = Point(3, 4);

    p += q;

    EXPECT_EQ(Point(4, 6), p);
}

TEST(MinusEqualOperatorTest, minus_equal_operator_test)
{
    Point p = Point(4, 6);
    Point q = Point(3, 4);

    p -= q;

    EXPECT_EQ(Point(1, 2), p);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/*
 * Test file for trespass functions
 */

#include "ai/navigator/trespass.h"

#include <gtest/gtest.h>

#include "geom/point.h"
#include "geom/rectangle.h"

TEST(TrespassTest, calcLinearTrespassScore_rectangle_test)
{
    Rectangle rect   = Rectangle(Point(0, 0), 6, 2);
    Point outside    = Point(-1, -2);
    Point corner     = Point(0, 0);
    Point edge_e     = Point(6, 1);
    Point edge_s     = Point(3, 0);
    Point centre     = Point(3, 1);
    Point quarter    = Point(1.5, 0.5);
    Point quarter_y  = Point(3, 0.5);
    Point two_thirds = Point(4, 0.2);

    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, outside), 0);
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, corner), 0);
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, edge_e), 0);
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, edge_s), 0);
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, centre), 1);
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, quarter), 0.5);
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, quarter_y), 1);
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, two_thirds), 2.0 / 3.0);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

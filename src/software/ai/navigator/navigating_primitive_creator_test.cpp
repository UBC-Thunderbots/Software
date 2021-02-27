#include "software/ai/navigator/navigating_primitive_creator.h"

#include <gtest/gtest.h>

class NavigatingPrimitiveCreatorTest : public testing::Test
{
   public:
    NavigatingPrimitiveCreatorTest()
        : navigating_primitive_creator(
                std::make_shared<const NavigatorConfig>())
    {
    }

    // The NavigatingPrimitiveCreator under test
    NavigatingPrimitiveCreator navigating_primitive_creator;
};

TEST_F(NavigatingPrimitiveCreatorTest,
       calculateTransitionSpeedBetweenSegments_tests_parallel_segments)
{
    Point testp1, testp2, testp3;
    double final_speed;
    // case 1
    testp1      = Point(1, 0);
    testp2      = Point(2, 0);
    testp3      = Point(3, 0);
    final_speed = 2.2;
    EXPECT_DOUBLE_EQ(final_speed,
                     navigating_primitive_creator.calculateTransitionSpeedBetweenSegments(
                         testp1, testp2, testp3, final_speed));

    // case 2
    testp1      = Point(1, 1);
    testp2      = Point(1, 2);
    testp3      = Point(1, 3);
    final_speed = -2.2;
    EXPECT_DOUBLE_EQ(final_speed,
                     navigating_primitive_creator.calculateTransitionSpeedBetweenSegments(
                         testp1, testp2, testp3, final_speed));
}

TEST_F(NavigatingPrimitiveCreatorTest,
       calculateTransitionSpeedBetweenSegments_tests_opposite_segments)
{
    Point testp1, testp2, testp3;
    double final_speed;
    // unequal segment length
    testp1      = Point(1, 0);
    testp2      = Point(2, 0);
    testp3      = Point(0, 0);
    final_speed = 0;
    EXPECT_DOUBLE_EQ(final_speed,
                     navigating_primitive_creator.calculateTransitionSpeedBetweenSegments(
                         testp1, testp2, testp3, final_speed));

    // equal segment length
    testp1      = Point(1, 1);
    testp2      = Point(1, 2);
    testp3      = Point(1, 1);
    final_speed = 0;
    EXPECT_DOUBLE_EQ(final_speed,
                     navigating_primitive_creator.calculateTransitionSpeedBetweenSegments(
                         testp1, testp2, testp3, final_speed));
}

TEST_F(NavigatingPrimitiveCreatorTest,
       calculateTransitionSpeedBetweenSegments_tests_perpendicular_segments)
{
    Point testp1, testp2, testp3;
    double final_speed;
    // case 1
    testp1      = Point(0, 1);
    testp2      = Point(1, 1);
    testp3      = Point(1, 2);
    final_speed = -2.2;
    EXPECT_DOUBLE_EQ(0,
                     navigating_primitive_creator.calculateTransitionSpeedBetweenSegments(
                         testp1, testp2, testp3, final_speed));

    // case 2
    testp1      = Point(1, 0);
    testp2      = Point(2, 0);
    testp3      = Point(2, 1);
    final_speed = 2.2;
    EXPECT_DOUBLE_EQ(0,
                     navigating_primitive_creator.calculateTransitionSpeedBetweenSegments(
                         testp1, testp2, testp3, final_speed));
}


TEST_F(NavigatingPrimitiveCreatorTest,
       calculateTransitionSpeedBetweenSegments_tests_nan_corner_cases)
{
    Point testp1, testp2, testp3;
    double final_speed;
    // case 1
    testp1      = Point(0, 1);
    testp2      = Point(0, 1);
    testp3      = Point(1, 2);
    final_speed = -2.2;
    EXPECT_FALSE(
        isnormal(navigating_primitive_creator.calculateTransitionSpeedBetweenSegments(
            testp1, testp2, testp3, final_speed)));

    // case 2
    testp1      = Point(1, 0);
    testp2      = Point(2, 0);
    testp3      = Point(2, 0);
    final_speed = 2.2;
    EXPECT_FALSE(
        isnormal(navigating_primitive_creator.calculateTransitionSpeedBetweenSegments(
            testp1, testp2, testp3, final_speed)));
}

#include "ai/navigator/placeholder_navigator/placeholder_navigator.h"

#include <gtest/gtest.h>

#include "test/test_util/test_util.h"

TEST(PlaceholderNavigatorTest, convert_catch_intent_to_catch_primitive)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    PlaceholderNavigator placeholderNavigator;

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<CatchIntent>(1, 0, 10, 0.3, 0));

    auto primitive_ptrs = placeholderNavigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = CatchPrimitive(1, 0, 10, 0.3);
    auto primitive          = dynamic_cast<CatchPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST(PlaceholderNavigatorTest, convert_chip_intent_to_chip_primitive)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    PlaceholderNavigator placeholderNavigator;

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<ChipIntent>(0, Point(), Angle::quarter(), 0, 1));

    auto primitive_ptrs = placeholderNavigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = ChipPrimitive(0, Point(), Angle::quarter(), 0);
    auto primitive          = dynamic_cast<ChipPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST(PlaceholderNavigatorTest,
     convert_direct_velocity_intent_to_direct_velocity_primitive)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    PlaceholderNavigator placeholderNavigator;

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<DirectVelocityIntent>(3, 1, -2, 0.4, 1000, 4));

    auto primitive_ptrs = placeholderNavigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = DirectVelocityPrimitive(3, 1, -2, 0.4, 1000);
    auto primitive = dynamic_cast<DirectVelocityPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST(PlaceholderNavigatorTest, convert_direct_wheels_intent_to_direct_wheels_primitive)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    PlaceholderNavigator placeholderNavigator;

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<DirectWheelsIntent>(2, 80, 22, 55, 201, 5000, 60));

    auto primitive_ptrs = placeholderNavigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = DirectWheelsPrimitive(2, 80, 22, 55, 201, 5000);
    auto primitive = dynamic_cast<DirectWheelsPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST(PlaceholderNavigatorTest, convert_dribble_intent_to_dribble_primitive)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    PlaceholderNavigator placeholderNavigator;

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<DribbleIntent>(0, Point(), Angle::quarter(), 8888, true, 50));

    auto primitive_ptrs = placeholderNavigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = DribblePrimitive(0, Point(), Angle::quarter(), 8888, true);
    auto primitive          = dynamic_cast<DribblePrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST(PlaceholderNavigatorTest, convert_kick_intent_to_kick_primitive)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    PlaceholderNavigator placeholderNavigator;

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<KickIntent>(0, Point(), Angle::quarter(), 0, 1));

    auto primitive_ptrs = placeholderNavigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = KickPrimitive(0, Point(), Angle::quarter(), 0);
    auto primitive          = dynamic_cast<KickPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST(PlaceholderNavigatorTest, convert_move_intent_to_move_primitive)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    PlaceholderNavigator placeholderNavigator;

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<MoveIntent>(0, Point(), Angle::quarter(), 0, 1));

    auto primitive_ptrs = placeholderNavigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = MovePrimitive(0, Point(), Angle::quarter(), 0);
    auto primitive          = dynamic_cast<MovePrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST(PlaceholderNavigatorTest, convert_movespin_intent_to_movespin_primitive)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    PlaceholderNavigator placeholderNavigator;

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<MoveSpinIntent>(0, Point(), AngularVelocity::full(), 1));

    auto primitive_ptrs = placeholderNavigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = MoveSpinPrimitive(0, Point(), AngularVelocity::full());
    auto primitive          = dynamic_cast<MoveSpinPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST(PlaceholderNavigatorTest, convert_pivot_intent_to_pivot_primitive)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    PlaceholderNavigator placeholderNavigator;

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<PivotIntent>(0, Point(1, 0.4), Angle::half(), 3.2, 1));

    auto primitive_ptrs = placeholderNavigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = PivotPrimitive(0, Point(1, 0.4), Angle::half(), 3.2);
    auto primitive          = dynamic_cast<PivotPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST(PlaceholderNavigatorTest, convert_stop_intent_to_stop_primitive)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    PlaceholderNavigator placeholderNavigator;

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<StopIntent>(0, false, 1));

    auto primitive_ptrs = placeholderNavigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = StopPrimitive(0, false);
    auto primitive          = dynamic_cast<StopPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST(PlaceholderNavigatorTest, convert_multiple_intents_to_primitives)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    PlaceholderNavigator placeholderNavigator;

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<StopIntent>(0, false, 1));
    intents.emplace_back(
        std::make_unique<PivotIntent>(0, Point(1, 0.4), Angle::half(), 3.2, 1));
    intents.emplace_back(
        std::make_unique<MoveIntent>(0, Point(), Angle::quarter(), 0, 1));

    auto primitive_ptrs = placeholderNavigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 3 primitives back
    EXPECT_EQ(primitive_ptrs.size(), 3);

    auto expected_stop_primitive = StopPrimitive(0, false);
    auto stop_primitive          = dynamic_cast<StopPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_stop_primitive, stop_primitive);
    auto expected_pivot_primitive = PivotPrimitive(0, Point(1, 0.4), Angle::half(), 3.2);
    auto pivot_primitive = dynamic_cast<PivotPrimitive &>(*(primitive_ptrs.at(1)));
    EXPECT_EQ(expected_pivot_primitive, pivot_primitive);
    auto expected_move_primitive = MovePrimitive(0, Point(), Angle::quarter(), 0);
    auto move_primitive          = dynamic_cast<MovePrimitive &>(*(primitive_ptrs.at(2)));
    EXPECT_EQ(expected_move_primitive, move_primitive);
}

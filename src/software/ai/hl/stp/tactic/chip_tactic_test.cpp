#include "software/ai/hl/stp/tactic/chip_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/action/chip_action.h"
#include "software/test_util/test_util.h"

TEST(ChipTacticTest, chip_45deg_from_destination)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(0, 1), 1.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(chip_action, nullptr);
    EXPECT_EQ(Point(0, 0), chip_action->getChipOrigin());
    EXPECT_EQ(Angle::quarter(), chip_action->getChipDirection());
    EXPECT_EQ(1.0, chip_action->getChipDistanceMeters());
}

TEST(ChipTacticTest, chip_to_same_point)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
            ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(0, 0), 2.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(chip_action, nullptr);
    EXPECT_EQ(Point(0, 0), chip_action->getChipOrigin());
    EXPECT_EQ(Angle::zero(), chip_action->getChipDirection());
    EXPECT_EQ(2.0, chip_action->getChipDistanceMeters());
}

TEST(ChipTacticTest, test_calculate_robot_cost)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
            ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(1, 0), 1.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(chip_action, nullptr);

    EXPECT_EQ(5 / world.field().totalXLength(), tactic.calculateRobotCost(robot, world));
}
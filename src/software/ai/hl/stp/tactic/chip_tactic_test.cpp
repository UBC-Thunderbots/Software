#include "software/ai/hl/stp/tactic/chip_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/action/chip_action.h"
#include "software/test_util/test_util.h"

bool compareChipActions(ChipAction *chip_action, ChipAction *expected_chip_action) {
    EXPECT_EQ(expected_chip_action->getChipOrigin(), chip_action->getChipOrigin());
    EXPECT_EQ(expected_chip_action->getChipDirection(), chip_action->getChipDirection());
    EXPECT_EQ(expected_chip_action->getChipDistanceMeters(), chip_action->getChipDistanceMeters());
}

TEST(ChipTacticTest, getChipDirection)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(0, 1), 1.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(chip_action, nullptr);
    EXPECT_EQ(Angle::quarter(), chip_action->getChipDirection());
}

TEST(ChipTacticTest, getChipOrigin)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(3, 1), Point(0, 1), 1.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(chip_action, nullptr);
    EXPECT_EQ(Point(3, 1), chip_action->getChipOrigin());
}

TEST(ChipTacticTest, getChipDistanceMeters)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(3, 1), Point(0, 1), 2.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(chip_action, nullptr);
    EXPECT_EQ(2.0, chip_action->getChipDistanceMeters());
}

TEST(ChipTacticTest, robot_behind_ball_chipping_towards_positive_x_positive_y)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(-0.3, 0), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    Ball ball({0, 0}, robot.position().toVector(), Timestamp::fromSeconds(0));

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(1, 1), 2.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    ASSERT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);

    ChipAction expected_chip_action = ChipAction();

    expected_chip_action.updateWorldParams(ball);
    expected_chip_action.updateControlParams(robot, ball.position(), Angle::fromDegrees(45.0), 2.0);

    // TODO implement compareChipActions
    // TODO cast action_ptr to ChipAction type

    ASSERT_NE(chip_action, nullptr);
//    EXPECT_EQ(Point(0, 0), chip_action->getChipOrigin());
//    EXPECT_EQ(Angle::fromDegrees(45.0), chip_action->getChipDirection());
//    EXPECT_EQ(2.0, chip_action->getChipDistanceMeters());
    EXPECT_TRUE(compareChipActions((ChipAction) &action_ptr, &expected_chip_action));
}

TEST(ChipTacticTest, robot_behind_ball_chipping_towards_negative_x_positive_y)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(-1.3, 2), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(-1, 1), 2.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(chip_action, nullptr);
    EXPECT_EQ(Point(0, 0), chip_action->getChipOrigin());
    EXPECT_EQ(Angle::fromDegrees(135.0), chip_action->getChipDirection());
    EXPECT_EQ(2.0, chip_action->getChipDistanceMeters());
}

TEST(ChipTacticTest, robot_behind_ball_chipping_towards_negative_x_negative_y)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(-0.5, 1.3), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(-1, -1), 2.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(chip_action, nullptr);
    EXPECT_EQ(Point(0, 0), chip_action->getChipOrigin());
    EXPECT_EQ(Angle::fromDegrees(225.0), chip_action->getChipDirection());
    EXPECT_EQ(2.0, chip_action->getChipDistanceMeters());
}

TEST(ChipTacticTest, robot_behind_ball_chipping_towards_positive_x_negative_y)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(-1.2, 2.1), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(1, -1), 2.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(chip_action, nullptr);
    EXPECT_EQ(Point(0, 0), chip_action->getChipOrigin());
    EXPECT_EQ(Angle::fromDegrees(315.0), chip_action->getChipDirection());
    EXPECT_EQ(2.0, chip_action->getChipDistanceMeters());
}

TEST(ChipTacticTest, robot_not_behind_ball_chipping_towards_positive_x_positive_y)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(0.3, 0), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(1, 1), 2.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(chip_action, nullptr);
    EXPECT_EQ(Point(0, 0), chip_action->getChipOrigin());
    EXPECT_EQ(Angle::fromDegrees(45.0), chip_action->getChipDirection());
    EXPECT_EQ(2.0, chip_action->getChipDistanceMeters());
}

TEST(ChipTacticTest, robot_not_behind_ball_chipping_towards_negative_x_positive_y)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(1.1, 0.2), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(-1, 1), 2.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(chip_action, nullptr);
    EXPECT_EQ(Point(0, 0), chip_action->getChipOrigin());
    EXPECT_EQ(Angle::fromDegrees(135.0), chip_action->getChipDirection());
    EXPECT_EQ(2.0, chip_action->getChipDistanceMeters());
}

TEST(ChipTacticTest, robot_not_behind_ball_chipping_towards_negative_x_negative_y)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(0.7, 2), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(-1, -1), 2.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(chip_action, nullptr);
    EXPECT_EQ(Point(0, 0), chip_action->getChipOrigin());
    EXPECT_EQ(Angle::fromDegrees(225.0), chip_action->getChipDirection());
    EXPECT_EQ(2.0, chip_action->getChipDistanceMeters());
}

TEST(ChipTacticTest, robot_not_behind_ball_chipping_towards_positive_x_negative_y)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(1.3, 1.2), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(1, -1), 2.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(chip_action, nullptr);
    EXPECT_EQ(Point(0, 0), chip_action->getChipOrigin());
    EXPECT_EQ(Angle::fromDegrees(315.0), chip_action->getChipDirection());
    EXPECT_EQ(2.0, chip_action->getChipDistanceMeters());
}

#include "software/ai/hl/stp/tactic/chip/chip_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/action/chip_action.h"
#include "software/test_util/test_util.h"

void compareChipActions(std::shared_ptr<ChipAction> chip_action,
                        std::shared_ptr<ChipAction> expected_chip_action)
{
    EXPECT_TRUE(TestUtil::equalWithinTolerance(expected_chip_action->getChipOrigin(),
                                               chip_action->getChipOrigin(), 1e-6));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(expected_chip_action->getChipDirection(),
                                               chip_action->getChipDirection(),
                                               Angle::fromDegrees(1e-6)));
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(expected_chip_action->getChipDistanceMeters(),
                                       chip_action->getChipDistanceMeters(), 1e-6));
}

TEST(ChipTacticTest, robot_behind_ball_chipping_towards_positive_x_positive_y)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(-0.3, 0), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    Ball ball({0, 0}, robot.position().toVector(), Timestamp::fromSeconds(0));

    tactic.updateWorldParams(world);
    EXPECT_EQ(tactic.getBall().position(), ball.position());

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(1, 1));
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    ASSERT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);

    std::shared_ptr<ChipAction> expected_chip_action = std::make_shared<ChipAction>();
    expected_chip_action->updateControlParams(robot, ball.position(),
                                              Angle::fromDegrees(45.0), sqrtf(2.0));

    ASSERT_NE(chip_action, nullptr);
    compareChipActions(chip_action, expected_chip_action);
}

TEST(ChipTacticTest, robot_behind_ball_chipping_towards_negative_x_positive_y)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(-1.3, 2), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    Ball ball({0, 0}, robot.position().toVector(), Timestamp::fromSeconds(0));

    tactic.updateWorldParams(world);
    EXPECT_EQ(tactic.getBall().position(), ball.position());

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(-1, 1));
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);

    std::shared_ptr<ChipAction> expected_chip_action = std::make_shared<ChipAction>();
    expected_chip_action->updateControlParams(robot, ball.position(),
                                              Angle::fromDegrees(135.0), sqrtf(2.0));

    ASSERT_NE(chip_action, nullptr);
    compareChipActions(chip_action, expected_chip_action);
}

TEST(ChipTacticTest, robot_behind_ball_chipping_towards_negative_x_negative_y)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(-0.5, 1.3), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    Ball ball({0, 0}, robot.position().toVector(), Timestamp::fromSeconds(0));

    tactic.updateWorldParams(world);
    EXPECT_EQ(tactic.getBall().position(), ball.position());

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(-1, -1));
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);

    std::shared_ptr<ChipAction> expected_chip_action = std::make_shared<ChipAction>();
    expected_chip_action->updateControlParams(robot, ball.position(),
                                              Angle::fromDegrees(225.0), sqrtf(2.0));

    ASSERT_NE(chip_action, nullptr);
    compareChipActions(chip_action, expected_chip_action);
}

TEST(ChipTacticTest, robot_behind_ball_chipping_towards_positive_x_negative_y)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(-1.2, 2.1), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    Ball ball({0, 0}, robot.position().toVector(), Timestamp::fromSeconds(0));

    tactic.updateWorldParams(world);
    EXPECT_EQ(tactic.getBall().position(), ball.position());

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(1, -1));
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);

    std::shared_ptr<ChipAction> expected_chip_action = std::make_shared<ChipAction>();
    expected_chip_action->updateControlParams(robot, ball.position(),
                                              Angle::fromDegrees(315.0), sqrtf(2.0));

    ASSERT_NE(chip_action, nullptr);
    compareChipActions(chip_action, expected_chip_action);
}

TEST(ChipTacticTest, robot_not_behind_ball_chipping_towards_positive_x_positive_y)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(0.3, 0), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    Ball ball({0, 0}, robot.position().toVector(), Timestamp::fromSeconds(0));

    tactic.updateWorldParams(world);
    EXPECT_EQ(tactic.getBall().position(), ball.position());

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(1, 1));
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);

    std::shared_ptr<ChipAction> expected_chip_action = std::make_shared<ChipAction>();
    expected_chip_action->updateControlParams(robot, ball.position(),
                                              Angle::fromDegrees(45.0), sqrtf(2.0));

    ASSERT_NE(chip_action, nullptr);
    compareChipActions(chip_action, expected_chip_action);
}

TEST(ChipTacticTest, robot_not_behind_ball_chipping_towards_negative_x_positive_y)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(1.1, 0.2), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    Ball ball({0, 0}, robot.position().toVector(), Timestamp::fromSeconds(0));

    tactic.updateWorldParams(world);
    EXPECT_EQ(tactic.getBall().position(), ball.position());

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(-1, 1));
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);

    std::shared_ptr<ChipAction> expected_chip_action = std::make_shared<ChipAction>();
    expected_chip_action->updateControlParams(robot, ball.position(),
                                              Angle::fromDegrees(135.0), sqrtf(2.0));

    ASSERT_NE(chip_action, nullptr);
    compareChipActions(chip_action, expected_chip_action);
}

TEST(ChipTacticTest, robot_not_behind_ball_chipping_towards_negative_x_negative_y)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(0.7, 2), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    Ball ball({0, 0}, robot.position().toVector(), Timestamp::fromSeconds(0));

    tactic.updateWorldParams(world);
    EXPECT_EQ(tactic.getBall().position(), ball.position());

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(-1, -1));
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);

    std::shared_ptr<ChipAction> expected_chip_action = std::make_shared<ChipAction>();
    expected_chip_action->updateControlParams(robot, ball.position(),
                                              Angle::fromDegrees(225.0), sqrtf(2.0));

    ASSERT_NE(chip_action, nullptr);
    compareChipActions(chip_action, expected_chip_action);
}

TEST(ChipTacticTest, robot_not_behind_ball_chipping_towards_positive_x_negative_y)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    Robot robot = Robot(0, Point(1.3, 1.2), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    Ball ball({0, 0}, robot.position().toVector(), Timestamp::fromSeconds(0));

    tactic.updateWorldParams(world);
    EXPECT_EQ(tactic.getBall().position(), ball.position());

    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Point(1, -1));
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);

    std::shared_ptr<ChipAction> expected_chip_action = std::make_shared<ChipAction>();
    expected_chip_action->updateControlParams(robot, ball.position(),
                                              Angle::fromDegrees(315.0), sqrtf(2.0));

    ASSERT_NE(chip_action, nullptr);
    compareChipActions(chip_action, expected_chip_action);
}

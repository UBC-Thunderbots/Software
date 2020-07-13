#include "software/ai/hl/stp/action/chip_action.h"

#include <gtest/gtest.h>

#include "software/ai/intent/chip_intent.h"
#include "software/ai/intent/move_intent.h"
#include "software/test_util/test_util.h"

TEST(ChipTacticTest, getChipOriginDirectionDistanceMeters)
{
    Robot robot = Robot(0, Point(-1.3, 2), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Ball ball({0, 0}, robot.position().toVector(), Timestamp::fromSeconds(0));

    std::shared_ptr<ChipAction> chip_action = std::make_shared<ChipAction>();
    chip_action->updateControlParams(robot, ball.position(), Angle::fromDegrees(45.0),
                                     2.0);

    ASSERT_NE(chip_action, nullptr);
    EXPECT_EQ(Point(0, 0), chip_action->getChipOrigin());
    EXPECT_EQ(Angle::fromDegrees(45.0), chip_action->getChipDirection());
    EXPECT_DOUBLE_EQ(2.0, chip_action->getChipDistanceMeters());
}

TEST(ChipActionTest, robot_behind_ball_chipping_towards_positive_x_positive_y)
{
    Robot robot(0, Point(-0.3, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball({0, 0}, robot.position().toVector(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    action.updateWorldParams(ball);
    action.updateControlParams(robot, ball.position(), Angle::zero(), 5.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        ChipIntent kick_intent = dynamic_cast<ChipIntent &>(*intent_ptr);
        EXPECT_EQ(0, kick_intent.getRobotId());
        EXPECT_EQ(Point(0, 0), kick_intent.getChipOrigin());
        EXPECT_EQ(Angle::zero(), kick_intent.getChipDirection());
        EXPECT_EQ(5.0, kick_intent.getChipDistance());
    }
    catch (...)
    {
        ADD_FAILURE() << "Chip intent not returned by Chip Action!";
    }
}

TEST(ChipActionTest, robot_behind_ball_chipping_towards_negative_x_positive_y)
{
    Robot robot(0, Point(-2.4, 2.3), Vector(), Angle::quarter(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball({-2.5, 2.5}, robot.position().toVector(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    action.updateWorldParams(ball);
    action.updateControlParams(robot, ball.position(), Angle::fromDegrees(105), 5.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        ChipIntent kick_intent = dynamic_cast<ChipIntent &>(*intent_ptr);
        EXPECT_EQ(0, kick_intent.getRobotId());
        EXPECT_EQ(Point(-2.5, 2.5), kick_intent.getChipOrigin());
        EXPECT_EQ(Angle::fromDegrees(105), kick_intent.getChipDirection());
        EXPECT_EQ(5.0, kick_intent.getChipDistance());
    }
    catch (...)
    {
        ADD_FAILURE() << "Chip intent not returned by Chip Action!";
    }
}

TEST(ChipActionTest, robot_behind_ball_chipping_towards_negative_x_negative_y)
{
    Robot robot(0, Point(0, 0), Vector(), Angle::quarter(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball({-0.05, -0.2}, robot.position().toVector(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    action.updateWorldParams(ball);
    action.updateControlParams(robot, ball.position(), Angle::fromDegrees(255), 3.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        ChipIntent kick_intent = dynamic_cast<ChipIntent &>(*intent_ptr);
        EXPECT_EQ(0, kick_intent.getRobotId());
        EXPECT_EQ(Point(-0.05, -0.2), kick_intent.getChipOrigin());
        EXPECT_EQ(Angle::fromDegrees(255), kick_intent.getChipDirection());
        EXPECT_EQ(3.0, kick_intent.getChipDistance());
    }
    catch (...)
    {
        ADD_FAILURE() << "Chip intent not returned by Chip Action!";
    }
}

TEST(ChipActionTest, robot_behind_ball_chipping_towards_positive_x_negative_y)
{
    Robot robot(0, Point(-0.125, 0.25), Vector(), Angle::threeQuarter(),
                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball({0, 0}, robot.position().toVector(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    action.updateWorldParams(ball);
    action.updateControlParams(robot, ball.position(), Angle::fromDegrees(306), 5.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        ChipIntent kick_intent = dynamic_cast<ChipIntent &>(*intent_ptr);
        EXPECT_EQ(0, kick_intent.getRobotId());
        EXPECT_EQ(Point(0, 0), kick_intent.getChipOrigin());
        EXPECT_EQ(Angle::fromDegrees(306), kick_intent.getChipDirection());
        EXPECT_EQ(5.0, kick_intent.getChipDistance());
    }
    catch (...)
    {
        ADD_FAILURE() << "Chip intent not returned by Chip Action!";
    }
}

TEST(ChipActionTest, robot_not_behind_ball_chipping_towards_positive_x_positive_y)
{
    Robot robot(0, Point(-1, 0.0), Vector(), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball({0, 0}, robot.position().toVector(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    action.updateWorldParams(ball);
    action.updateControlParams(robot, ball.position(), Angle::zero(), 5.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_EQ(0, move_intent.getRobotId());
        // Check the MoveIntent is moving roughly behind the ball
        EXPECT_TRUE(TestUtil::equalWithinTolerance(move_intent.getDestination(),
                                                   Point(-0.18, 0), 0.1));
        EXPECT_EQ(Angle::zero(), move_intent.getFinalAngle());
        EXPECT_EQ(0.0, move_intent.getFinalSpeed());
    }
    catch (...)
    {
        ADD_FAILURE() << "Move intent not returned by Chip Action!";
    }
}

TEST(ChipActionTest, robot_not_behind_ball_chipping_towards_negative_x_positive_y)
{
    Robot robot(0, Point(-2, 5), Vector(), Angle::quarter(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball({-2.5, 2.5}, robot.position().toVector(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    action.updateWorldParams(ball);
    action.updateControlParams(robot, ball.position(), Angle::fromDegrees(105), 5.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_EQ(0, move_intent.getRobotId());
        // Check the MoveIntent is moving roughly behind the ball
        EXPECT_TRUE(TestUtil::equalWithinTolerance(move_intent.getDestination(),
                                                   Point(-2.45, 2.25), 0.1));
        EXPECT_EQ(Angle::fromDegrees(105), move_intent.getFinalAngle());
        EXPECT_EQ(0.0, move_intent.getFinalSpeed());
    }
    catch (...)
    {
        ADD_FAILURE() << "Move intent not returned by Chip Action!";
    }
}

TEST(ChipActionTest, robot_not_behind_ball_chipping_towards_negative_x_negative_y)
{
    Robot robot(0, Point(0, 0), Vector(), Angle::quarter(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Ball ball({-1, -4}, robot.position().toVector(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    action.updateWorldParams(ball);
    action.updateControlParams(robot, ball.position(), Angle::fromDegrees(255), 3.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_EQ(0, move_intent.getRobotId());
        // Check the MoveIntent is moving roughly behind the ball
        EXPECT_TRUE(TestUtil::equalWithinTolerance(move_intent.getDestination(),
                                                   Point(-0.85, -3.75), 0.1));
        EXPECT_EQ(Angle::fromDegrees(255), move_intent.getFinalAngle());
        EXPECT_EQ(0.0, move_intent.getFinalSpeed());
    }
    catch (...)
    {
        ADD_FAILURE() << "Move intent not returned by Chip Action!";
    }
}

TEST(ChipActionTest, robot_not_behind_ball_chipping_towards_positive_x_negative_y)
{
    Robot robot(0, Point(0.5, 1), Vector(), Angle::threeQuarter(),
                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball({0, 0}, robot.position().toVector(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    action.updateWorldParams(ball);
    action.updateControlParams(robot, ball.position(), Angle::fromDegrees(306), 5.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_EQ(0, move_intent.getRobotId());
        // Check the MoveIntent is moving roughly behind the ball
        EXPECT_TRUE(TestUtil::equalWithinTolerance(move_intent.getDestination(),
                                                   Point(-0.15, 0.25), 0.1));
        EXPECT_EQ(Angle::fromDegrees(306), move_intent.getFinalAngle());
        EXPECT_EQ(0.0, move_intent.getFinalSpeed());
    }
    catch (...)
    {
        ADD_FAILURE() << "Move intent not returned by Chip Action!";
    }
}

#include "ai/hl/stp/action/chip_action.h"

#include <gtest/gtest.h>

#include "ai/intent/chip_intent.h"
#include "ai/intent/move_intent.h"

TEST(ChipActionTest, robot_behind_ball_chipping_towards_positive_x_positive_y)
{
    Robot robot       = Robot(0, Point(-0.5, 0), Vector(), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    auto intent_ptr =
        action.updateStateAndGetNextIntent(robot, Point(0, 0), Angle::zero(), 5.0);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    ChipIntent chip_intent = dynamic_cast<ChipIntent &>(*intent_ptr);
    EXPECT_EQ(0, chip_intent.getRobotId());
    EXPECT_EQ(Point(0, 0), chip_intent.getChipOrigin());
    EXPECT_EQ(Angle::zero(), chip_intent.getChipDirection());
    EXPECT_EQ(5.0, chip_intent.getChipDistance());
}

TEST(ChipActionTest, robot_behind_ball_chipping_towards_negative_x_positive_y)
{
    Robot robot       = Robot(0, Point(-2.3, 2.1), Vector(), Angle::quarter(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    auto intent_ptr = action.updateStateAndGetNextIntent(robot, Point(-2.5, 2.5),
                                                         Angle::ofDegrees(105), 5.0);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    ChipIntent chip_intent = dynamic_cast<ChipIntent &>(*intent_ptr);
    EXPECT_EQ(0, chip_intent.getRobotId());
    EXPECT_EQ(Point(-2.5, 2.5), chip_intent.getChipOrigin());
    EXPECT_EQ(Angle::ofDegrees(105), chip_intent.getChipDirection());
    EXPECT_EQ(5.0, chip_intent.getChipDistance());
}

TEST(ChipActionTest, robot_behind_ball_chipping_towards_negative_x_negative_y)
{
    Robot robot       = Robot(0, Point(0, 0), Vector(), Angle::quarter(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    auto intent_ptr = action.updateStateAndGetNextIntent(robot, Point(-0.1, -0.4),
                                                         Angle::ofDegrees(255), 3.0);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    ChipIntent chip_intent = dynamic_cast<ChipIntent &>(*intent_ptr);
    EXPECT_EQ(0, chip_intent.getRobotId());
    EXPECT_EQ(Point(-0.1, -0.4), chip_intent.getChipOrigin());
    EXPECT_EQ(Angle::ofDegrees(255), chip_intent.getChipDirection());
    EXPECT_EQ(3.0, chip_intent.getChipDistance());
}

TEST(ChipActionTest, robot_behind_ball_chipping_towards_positive_x_negative_y)
{
    Robot robot       = Robot(0, Point(-0.25, 0.5), Vector(), Angle::threeQuarter(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    auto intent_ptr = action.updateStateAndGetNextIntent(robot, Point(0, 0),
                                                         Angle::ofDegrees(306), 5.0);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    ChipIntent chip_intent = dynamic_cast<ChipIntent &>(*intent_ptr);
    EXPECT_EQ(0, chip_intent.getRobotId());
    EXPECT_EQ(Point(0, 0), chip_intent.getChipOrigin());
    EXPECT_EQ(Angle::ofDegrees(306), chip_intent.getChipDirection());
    EXPECT_EQ(5.0, chip_intent.getChipDistance());
}

TEST(ChipActionTest, robot_not_behind_ball_chipping_towards_positive_x_positive_y)
{
    Robot robot       = Robot(0, Point(-1, 0.0), Vector(), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    auto intent_ptr =
        action.updateStateAndGetNextIntent(robot, Point(0, 0), Angle::zero(), 5.0);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
    EXPECT_EQ(0, move_intent.getRobotId());
    // Check the MoveIntent is moving roughly behind the ball
    EXPECT_TRUE(move_intent.getDestination().isClose(Point(-0.28, 0), 0.1));
    EXPECT_EQ(Angle::zero(), move_intent.getFinalAngle());
    EXPECT_EQ(0.0, move_intent.getFinalSpeed());
}

TEST(ChipActionTest, robot_not_behind_ball_chipping_towards_negative_x_positive_y)
{
    Robot robot       = Robot(0, Point(-2, 5), Vector(), Angle::quarter(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    auto intent_ptr = action.updateStateAndGetNextIntent(robot, Point(-2.5, 2.5),
                                                         Angle::ofDegrees(105), 5.0);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
    EXPECT_EQ(0, move_intent.getRobotId());
    // Check the MoveIntent is moving roughly behind the ball
    EXPECT_TRUE(move_intent.getDestination().isClose(Point(-2.45, 2.25), 0.1));
    EXPECT_EQ(Angle::ofDegrees(105), move_intent.getFinalAngle());
    EXPECT_EQ(0.0, move_intent.getFinalSpeed());
}

TEST(ChipActionTest, robot_not_behind_ball_chipping_towards_negative_x_negative_y)
{
    Robot robot       = Robot(0, Point(0, 0), Vector(), Angle::quarter(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    auto intent_ptr = action.updateStateAndGetNextIntent(robot, Point(-1, -4),
                                                         Angle::ofDegrees(255), 3.0);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
    EXPECT_EQ(0, move_intent.getRobotId());
    // Check the MoveIntent is moving roughly behind the ball
    EXPECT_TRUE(move_intent.getDestination().isClose(Point(-0.85, -3.75), 0.1));
    EXPECT_EQ(Angle::ofDegrees(255), move_intent.getFinalAngle());
    EXPECT_EQ(0.0, move_intent.getFinalSpeed());
}

TEST(ChipActionTest, robot_not_behind_ball_chipping_towards_positive_x_negative_y)
{
    Robot robot       = Robot(0, Point(0.5, 1), Vector(), Angle::threeQuarter(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    ChipAction action = ChipAction();

    auto intent_ptr = action.updateStateAndGetNextIntent(robot, Point(0, 0),
                                                         Angle::ofDegrees(306), 5.0);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
    EXPECT_EQ(0, move_intent.getRobotId());
    // Check the MoveIntent is moving roughly behind the ball
    EXPECT_TRUE(move_intent.getDestination().isClose(Point(-0.15, 0.25), 0.1));
    EXPECT_EQ(Angle::ofDegrees(306), move_intent.getFinalAngle());
    EXPECT_EQ(0.0, move_intent.getFinalSpeed());
}

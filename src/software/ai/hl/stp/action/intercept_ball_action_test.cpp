#include "software/ai/hl/stp/action/intercept_ball_action.h"

#include <gtest/gtest.h>

#include "software/ai/intent/move_intent.h"
#include "software/test_util/test_util.h"
#include "software/world/ball.h"

TEST(InterceptBallActionTest, test_robot_ahead_of_ball_moves_in_front_of_ball)
{
    Field field = ::TestUtil::createSSLDivBField();
    Ball ball   = Ball(Point(0, 0), Vector(1, 0), Timestamp::fromSeconds(0));
    Robot robot = Robot(0, Point(3, 1), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));


    InterceptBallAction action = InterceptBallAction(field, ball, true);

    action.updateWorldParams(field, ball);
    action.updateControlParams(robot);
    std::unique_ptr<Intent> intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(Point(3, 0).isClose(move_intent.getDestination(), 0.01));
        Angle angle_facing_ball = (ball.position() - robot.position()).orientation();
        EXPECT_EQ(angle_facing_ball, move_intent.getFinalAngle());
        EXPECT_EQ(AutokickType::NONE, move_intent.getAutoKickType());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the InterceptBallAction";
    }
}

TEST(InterceptBallActionTest, test_robot_moves_to_edge_of_field_if_ball_moving_too_fast)
{
    Field field = ::TestUtil::createSSLDivBField();
    Ball ball   = Ball(Point(0, 0), Vector(8, 0), Timestamp::fromSeconds(0));
    Robot robot = Robot(0, Point(3, 1), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));


    InterceptBallAction action = InterceptBallAction(field, ball, true);

    action.updateWorldParams(field, ball);
    action.updateControlParams(robot);
    std::unique_ptr<Intent> intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(field.enemyGoalCenter().isClose(move_intent.getDestination(), 0.01));
        Angle angle_facing_ball = (ball.position() - robot.position()).orientation();
        EXPECT_EQ(angle_facing_ball, move_intent.getFinalAngle());
        EXPECT_EQ(AutokickType::NONE, move_intent.getAutoKickType());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the InterceptBallAction";
    }
}

TEST(InterceptBallActionTest, test_robot_moves_to_the_ball_if_the_ball_is_moving_slowly)
{
    Field field = ::TestUtil::createSSLDivBField();
    Ball ball   = Ball(Point(0, 0), Vector(0.2, 0), Timestamp::fromSeconds(0));
    Robot robot = Robot(0, Point(3, 1), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));


    InterceptBallAction action = InterceptBallAction(field, ball, true);

    action.updateWorldParams(field, ball);
    action.updateControlParams(robot);
    std::unique_ptr<Intent> intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(ball.position().isClose(move_intent.getDestination(), 0.01));
        Angle angle_facing_ball = (ball.position() - robot.position()).orientation();
        EXPECT_EQ(angle_facing_ball, move_intent.getFinalAngle());
        EXPECT_EQ(AutokickType::NONE, move_intent.getAutoKickType());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the InterceptBallAction";
    }
}

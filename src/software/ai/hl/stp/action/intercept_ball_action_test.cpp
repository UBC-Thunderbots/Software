#include "software/ai/hl/stp/action/intercept_ball_action.h"

#include <gtest/gtest.h>

#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/segment.h"
#include "software/test_util/test_util.h"
#include "software/world/ball.h"

TEST(InterceptBallActionTest, intercept_fast_moving_ball)
{
    Ball ball   = Ball(Point(-3, 0), Vector(2, 0), Timestamp::fromSeconds(0));
    Robot robot = Robot(0, Point(3, 1), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, {-3, 0}, Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(2, 0), Timestamp::fromSeconds(0));

    InterceptBallAction action = InterceptBallAction(world.field(), world.ball());

    action.updateWorldParams(world);
    action.updateControlParams(robot);
    std::unique_ptr<Intent> intent_ptr = action.getNextIntent();

    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        // The robot should move to some point ahead of the ball
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        Segment ball_trajectory(Point(-1, 0), Point(4, 0));
        EXPECT_LT(distance(move_intent.getDestination(), ball_trajectory), 0.01);

        // The robot should immediately try facing the direction the ball is coming from
        EXPECT_EQ((-ball.velocity()).orientation(), move_intent.getFinalAngle());
    }
    catch (std::bad_cast &)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the InterceptBallAction";
    }
}

TEST(InterceptBallActionTest, ball_moving_too_fast_to_intercept_within_field)
{
    Robot robot = Robot(0, Point(3, 1), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(-10, 0), Timestamp::fromSeconds(0));

    InterceptBallAction action = InterceptBallAction(world.field(), world.ball());

    action.updateWorldParams(world);
    action.updateControlParams(robot);
    std::unique_ptr<Intent> intent_ptr = action.getNextIntent();

    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        // The robot should move to approximately where the ball will leave the field
        EXPECT_TRUE(TestUtil::equalWithinTolerance(world.field().friendlyGoalCenter(),
                                                   move_intent.getDestination(), 0.15));

        EXPECT_EQ((-world.ball().velocity()).orientation(), move_intent.getFinalAngle());
    }
    catch (std::bad_cast &)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the InterceptBallAction";
    }
}

TEST(InterceptBallActionTest, intercept_slow_moving_ball)
{
    Robot robot = Robot(0, Point(3, 1), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0.1, 0), Timestamp::fromSeconds(0));

    InterceptBallAction action = InterceptBallAction(world.field(), world.ball());

    action.updateWorldParams(world);
    action.updateControlParams(robot);
    std::unique_ptr<Intent> intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(world.ball().position(),
                                                   move_intent.getDestination(), 0.01));
        Angle angle_facing_ball =
            (world.ball().position() - robot.position()).orientation();
        EXPECT_EQ(angle_facing_ball, move_intent.getFinalAngle());
    }
    catch (std::bad_cast &)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the InterceptBallAction";
    }
}

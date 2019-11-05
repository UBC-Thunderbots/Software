#include "software/ai/hl/stp/tactic/shadow_freekicker_tactic.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/line.h"
#include "software/geom/util.h"
#include "software/test_util/test_util.h"

TEST(ShadowFreekickerTacticTest, test_shadow_free_kicker_left_side)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(
        world, {0, Point(world.field().friendlyCornerPos().y(), 0)},
        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallPosition(
        world, Point(-0.09, world.field().friendlyCornerPos().y() - 0.09),
        Timestamp::fromSeconds(0));
    world =
        ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Robot friendly_robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot});

    ShadowFreekickerTactic tactic =
        ShadowFreekickerTactic(ShadowFreekickerTactic::LEFT, world.enemyTeam(),
                               world.ball(), world.field(), false);
    tactic.updateRobot(friendly_robot);

    auto intent_ptr = tactic.getNextIntent();

    ASSERT_TRUE(intent_ptr);

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        // The edge of the robot should be slightly more than 0.5m from the ball
        EXPECT_NEAR((world.ball().position() - move_intent.getDestination()).length(),
                    0.62, 0.05);

        // The robot should be just to the left of the line between the friendly net and
        // the ball (from the POV of the friendly net)
        Line ball_to_net_line =
            Line(world.ball().position(), world.field().friendlyGoal());
        EXPECT_NEAR(dist(ball_to_net_line, move_intent.getDestination()), 0.09, 0.01);
        Angle goal_to_ball_angle =
            (world.ball().position() - world.field().friendlyGoal()).orientation();
        Angle goal_to_dest_angle =
            (move_intent.getDestination() - world.field().friendlyGoal()).orientation();
        EXPECT_GT(goal_to_dest_angle, goal_to_ball_angle);
    }
    catch (std::bad_cast &)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the ShootGoalTactic!";
    }
}

TEST(ShadowFreekickerTacticTest, test_shadow_free_kicker_right_side)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(
        world, {0, Point(world.field().friendlyCornerPos().y(), 0)},
        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallPosition(
        world, Point(-0.09, world.field().friendlyCornerPos().y() - 0.09),
        Timestamp::fromSeconds(0));
    world =
        ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Robot friendly_robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot});

    ShadowFreekickerTactic tactic =
        ShadowFreekickerTactic(ShadowFreekickerTactic::RIGHT, world.enemyTeam(),
                               world.ball(), world.field(), false);
    tactic.updateRobot(friendly_robot);

    auto intent_ptr = tactic.getNextIntent();

    ASSERT_TRUE(intent_ptr);

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        // The edge of the robot should be slightly more than 0.5m from the ball
        EXPECT_NEAR((world.ball().position() - move_intent.getDestination()).length(),
                    0.62, 0.05);

        // The robot should be just to the right of the line between the friendly net and
        // the ball (from the POV of the friendly net)
        Line ball_to_net_line =
            Line(world.ball().position(), world.field().friendlyGoal());
        EXPECT_NEAR(dist(ball_to_net_line, move_intent.getDestination()), 0.09, 0.01);
        Angle goal_to_ball_angle =
            (world.ball().position() - world.field().friendlyGoal()).orientation();
        Angle goal_to_dest_angle =
            (move_intent.getDestination() - world.field().friendlyGoal()).orientation();
        EXPECT_LT(goal_to_dest_angle, goal_to_ball_angle);
    }
    catch (std::bad_cast &)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the ShootGoalTactic!";
    }
}

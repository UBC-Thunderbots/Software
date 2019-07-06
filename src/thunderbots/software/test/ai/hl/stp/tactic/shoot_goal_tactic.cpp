#include "ai/hl/stp/tactic/shoot_goal_tactic.h"

#include <gtest/gtest.h>

#include "ai/intent/chip_intent.h"
#include "ai/intent/kick_intent.h"
#include "ai/intent/move_intent.h"
#include "test/test_util/test_util.h"

TEST(ShootGoalTacticTest, robot_will_shoot_on_open_net)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});
    Ball ball(Point(ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    world.updateBallState(ball);

    ShootGoalTactic tactic =
        ShootGoalTactic(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball(), Angle::zero(), std::nullopt, false);
    tactic.updateRobot(robot);
    tactic.updateParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball());

    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(intent_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_TRUE(tactic.hasShotAvailable());

    try
    {
        KickIntent kick_intent = dynamic_cast<KickIntent &>(*intent_ptr);
        EXPECT_EQ(0, kick_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE() << "KickIntent was not returned by the ShootGoalTactic!";
    }
}

TEST(ShootGoalTacticTest, robot_will_commit_to_a_shot_until_it_is_entirely_blocked)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(4.5, 0.25)},
                                                     Timestamp::fromSeconds(0));
    Ball ball(Point(ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    world.updateBallState(ball);

    ShootGoalTactic tactic =
        ShootGoalTactic(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball(), Angle::ofDegrees(6), std::nullopt, false);
    tactic.updateRobot(robot);
    tactic.updateParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball());

    // The robot will start the shot since it can see enough of the net
    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(intent_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_TRUE(tactic.hasShotAvailable());

    try
    {
        KickIntent kick_intent = dynamic_cast<KickIntent &>(*intent_ptr);
        EXPECT_EQ(0, kick_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE() << "KickIntent was not returned by the ShootGoalTactic!";
    }

    // The enemy has moved up to block more than 0.5 of the net, but since the net is not
    // entirely blocked the robot will still try shoot
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(4.2, 0)},
                                                     Timestamp::fromSeconds(0));
    tactic.updateRobot(robot);
    tactic.updateParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball());
    intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(intent_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_TRUE(tactic.hasShotAvailable());

    try
    {
        KickIntent kick_intent = dynamic_cast<KickIntent &>(*intent_ptr);
        EXPECT_EQ(0, kick_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE() << "KickIntent was not returned by the ShootGoalTactic!";
    }

    // The net is now entirely blocked (but the enemy robot is not quite yet in danger of
    // taking the ball), so the friendly robot just tries to line up to the ball
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.7, 0)},
                                                     Timestamp::fromSeconds(0));
    tactic.updateRobot(robot);
    tactic.updateParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball());
    intent_ptr = tactic.getNextIntent();
    intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(intent_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_FALSE(tactic.hasShotAvailable());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_EQ(0, move_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the ShootGoalTactic!";
    }
}

TEST(ShootGoalTacticTest, robot_will_align_to_ball_if_shot_is_blocked)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(1, 0)},
                                                     Timestamp::fromSeconds(0));
    Ball ball(Point(ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    world.updateBallState(ball);

    ShootGoalTactic tactic =
        ShootGoalTactic(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball(), Angle::ofDegrees(1.27), std::nullopt, false);
    tactic.updateRobot(robot);
    tactic.updateParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball());

    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(intent_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_FALSE(tactic.hasShotAvailable());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_EQ(0, move_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the ShootGoalTactic!";
    }
}

TEST(ShootGoalTacticTest,
     robot_will_chip_ball_if_enemy_close_to_stealing_ball_enemy_in_front)
{
    // Enemy robot directly in front of the friendly, at the very far end of the
    // "enemy close" bounding box

    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::ofDegrees(320),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});

    Vector robot_orientation_vec = Vector::createFromAngle(robot.orientation());
    Point enemy_position = robot_orientation_vec.norm(2.5 * ROBOT_MAX_RADIUS_METERS);
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {enemy_position},
                                                     Timestamp::fromSeconds(0));
    Ball ball(robot.position() + robot_orientation_vec.norm(0.05), Vector(0, 0),
              Timestamp::fromSeconds(0));
    world.updateBallState(ball);

    Point fallback_chip_target = robot.position() + robot_orientation_vec.norm(1);
    ShootGoalTactic tactic =
        ShootGoalTactic(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball(), Angle::ofDegrees(1.27), fallback_chip_target, false);
    tactic.updateRobot(robot);
    tactic.updateParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball());

    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(intent_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_FALSE(tactic.hasShotAvailable());

    try
    {
        ChipIntent chip_intent = dynamic_cast<ChipIntent &>(*intent_ptr);
        EXPECT_EQ(0, chip_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE() << "ChipIntent was not returned by the ShootGoalTactic!";
    }
}

TEST(ShootGoalTacticTest,
     robot_will_chip_ball_if_enemy_close_to_stealing_ball_enemy_off_to_one_side)
{
    // Enemy off to one side of the shooter, but just inside the "enemy close"
    // bounding box

    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(1, -3), Vector(0, 0), Angle::ofDegrees(320),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});

    Vector robot_orientation_vec = Vector::createFromAngle(robot.orientation());

    Point enemy_position = robot.position() +
                           robot_orientation_vec.perp().norm(-ROBOT_MAX_RADIUS_METERS) +
                           robot_orientation_vec.norm(0.05);
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {enemy_position},
                                                     Timestamp::fromSeconds(0));
    Ball ball(robot.position() + robot_orientation_vec.norm(0.05), Vector(0, 0),
              Timestamp::fromSeconds(0));
    world.updateBallState(ball);

    Point fallback_chip_target = robot.position() + robot_orientation_vec.norm(1);
    ShootGoalTactic tactic     = ShootGoalTactic(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        Angle::ofDegrees(1.27), fallback_chip_target, false);
    tactic.updateRobot(robot);
    tactic.updateParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball());

    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(intent_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_FALSE(tactic.hasShotAvailable());

    try
    {
        ChipIntent chip_intent = dynamic_cast<ChipIntent &>(*intent_ptr);
        EXPECT_EQ(0, chip_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE() << "ChipIntent was not returned by the ShootGoalTactic!";
    }
}

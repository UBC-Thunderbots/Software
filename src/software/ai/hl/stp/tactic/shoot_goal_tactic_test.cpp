#include "software/ai/hl/stp/tactic/shoot_goal_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/action/chip_action.h"
#include "software/ai/hl/stp/action/kick_action.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/test_util/test_util.h"

TEST(ShootGoalTacticTest, robot_will_shoot_on_open_net)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});
    BallState ballState(Point(ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0),
                        Timestamp::fromSeconds(0));
    world.updateBallState(ballState);

    ShootGoalTactic tactic =
        ShootGoalTactic(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball(), Angle::zero(), std::nullopt, false);
    tactic.updateRobot(robot);
    tactic.updateWorldParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                             world.ball());

    auto action_ptr = tactic.getNextAction();

    // Check an intent was returned (the pointer is not null)
    ASSERT_NE(nullptr, action_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_TRUE(tactic.hasShotAvailable());

    auto kick_action = std::dynamic_pointer_cast<KickAction>(action_ptr);
    ASSERT_NE(nullptr, kick_action);
    ASSERT_TRUE(kick_action->getRobot().has_value());
    EXPECT_EQ(0, kick_action->getRobot()->id());
}

TEST(ShootGoalTacticTest, robot_will_commit_to_a_shot_until_it_is_entirely_blocked)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(4.5, 0.25)},
                                                     Timestamp::fromSeconds(0));
    BallState ballState(Point(ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0),
                        Timestamp::fromSeconds(0));
    world.updateBallState(ballState);

    ShootGoalTactic tactic =
        ShootGoalTactic(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball(), Angle::fromDegrees(6), std::nullopt, false);
    tactic.updateRobot(robot);
    tactic.updateWorldParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                             world.ball());

    // The robot will start the shot since it can see enough of the net
    auto action_ptr = tactic.getNextAction();

    // Check an intent was returned (the pointer is not null)
    ASSERT_NE(nullptr, action_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_TRUE(tactic.hasShotAvailable());

    auto kick_action = std::dynamic_pointer_cast<KickAction>(action_ptr);
    ASSERT_NE(nullptr, kick_action);
    ASSERT_TRUE(kick_action->getRobot().has_value());
    EXPECT_EQ(0, kick_action->getRobot()->id());

    // The enemy has moved up to block more than 0.5 of the net, but since the net is not
    // entirely blocked the robot will still try shoot
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(4.2, 0)},
                                                     Timestamp::fromSeconds(0));
    tactic.updateRobot(robot);
    tactic.updateWorldParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                             world.ball());
    action_ptr = tactic.getNextAction();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(action_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_TRUE(tactic.hasShotAvailable());

    kick_action = std::dynamic_pointer_cast<KickAction>(action_ptr);
    ASSERT_NE(nullptr, kick_action);
    ASSERT_TRUE(kick_action->getRobot().has_value());
    EXPECT_EQ(0, kick_action->getRobot()->id());

    // The net is now entirely blocked (but the enemy robot is not quite yet in danger of
    // taking the ball), so the friendly robot just tries to line up to the ball
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.7, 0)},
                                                     Timestamp::fromSeconds(0));
    tactic.updateRobot(robot);
    tactic.updateWorldParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                             world.ball());
    action_ptr = tactic.getNextAction();
    action_ptr = tactic.getNextAction();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(action_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_FALSE(tactic.hasShotAvailable());


    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    ASSERT_TRUE(move_action->getRobot().has_value());
    EXPECT_EQ(0, move_action->getRobot()->id());
}

TEST(ShootGoalTacticTest, robot_will_align_to_ball_if_shot_is_blocked)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(1, 0)},
                                                     Timestamp::fromSeconds(0));
    BallState ballState(Point(ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0),
                        Timestamp::fromSeconds(0));
    world.updateBallState(ballState);

    ShootGoalTactic tactic =
        ShootGoalTactic(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball(), Angle::fromDegrees(1.27), std::nullopt, false);
    tactic.updateRobot(robot);
    tactic.updateWorldParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                             world.ball());

    auto action_ptr = tactic.getNextAction();

    // Check an intent was returned (the pointer is not null)
    ASSERT_NE(nullptr, action_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_FALSE(tactic.hasShotAvailable());

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    ASSERT_TRUE(move_action->getRobot().has_value());
    EXPECT_EQ(0, move_action->getRobot()->id());
}

TEST(ShootGoalTacticTest, robot_will_chip_ball_if_enemy_close_to_stealing_ball)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.25, 0)},
                                                     Timestamp::fromSeconds(0));
    BallState ballState(Point(ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0),
                        Timestamp::fromSeconds(0));
    world.updateBallState(ballState);

    ShootGoalTactic tactic =
        ShootGoalTactic(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball(), Angle::fromDegrees(1.27), std::nullopt, false);
    tactic.updateRobot(robot);
    tactic.updateWorldParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                             world.ball());

    auto action_ptr = tactic.getNextAction();

    // Check an intent was returned (the pointer is not null)
    ASSERT_NE(nullptr, action_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_FALSE(tactic.hasShotAvailable());

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(nullptr, chip_action);
    ASSERT_TRUE(chip_action->getRobot().has_value());
    EXPECT_EQ(0, chip_action->getRobot()->id());
}

TEST(ShootGoalTacticTest, test_calculate_robot_cost_when_robot_close_to_ball)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});

    BallState ballState(Point(0.5, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    world.updateBallState(ballState);

    ShootGoalTactic tactic =
        ShootGoalTactic(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball(), Angle::fromDegrees(1.27), std::nullopt, false);

    double cost          = tactic.calculateRobotCost(robot, world);
    double expected_cost = 0.0520833333;
    EXPECT_NEAR(cost, expected_cost, 1e-6);
}

TEST(ShootGoalTacticTest, test_calculate_robot_cost_when_robot_far_from_ball)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});

    BallState ballState(Point(3, -2.5), Vector(0, 0), Timestamp::fromSeconds(0));
    world.updateBallState(ballState);

    ShootGoalTactic tactic =
        ShootGoalTactic(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball(), Angle::fromDegrees(1.27), std::nullopt, false);

    double cost          = tactic.calculateRobotCost(robot, world);
    double expected_cost = 0.40678383728;
    EXPECT_NEAR(cost, expected_cost, 1e-6);
}

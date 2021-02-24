#include "software/ai/hl/stp/tactic/shoot_goal_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/action/chip_action.h"
#include "software/ai/hl/stp/action/intercept_ball_action.h"
#include "software/ai/hl/stp/action/kick_action.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/test_util/test_util.h"

class ShootGoalTacticTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        shoot_goal_tactic = std::make_shared<ShootGoalTacticConfig>();
    }

    std::shared_ptr<ShootGoalTacticConfig> shoot_goal_tactic;
}


TEST(ShootGoalTacticTest, robot_will_shoot_on_open_net)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.updateFriendlyTeamState(Team({robot}));
    BallState ball_state(Point(ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0));
    world.updateBall(Ball(ball_state, Timestamp::fromSeconds(0)));

    ShootGoalTactic tactic = ShootGoalTactic(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        Angle::zero(), std::nullopt, false, shoot_goal_tactic);
    tactic.updateRobot(robot);
    tactic.updateWorldParams(world);

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
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.updateFriendlyTeamState(Team({robot}));
    world = ::TestUtil::setEnemyRobotPositions(world, {Point(4.5, 0.25)},
                                               Timestamp::fromSeconds(0));
    BallState ball_state(Point(ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0));
    world.updateBall(Ball(ball_state, Timestamp::fromSeconds(0)));

    ShootGoalTactic tactic = ShootGoalTactic(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        Angle::fromDegrees(4), std::nullopt, false, shoot_goal_tactic);
    tactic.updateRobot(robot);
    tactic.updateWorldParams(world);

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
    world = ::TestUtil::setEnemyRobotPositions(world, {Point(4.2, 0)},
                                               Timestamp::fromSeconds(0));
    tactic.updateRobot(robot);
    tactic.updateWorldParams(world);
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
    // taking the ball), so the friendly robot just tries to intercept
    world = ::TestUtil::setEnemyRobotPositions(world, {Point(0.7, 0)},
                                               Timestamp::fromSeconds(0));
    tactic.updateRobot(robot);
    tactic.updateWorldParams(world);
    action_ptr = tactic.getNextAction();
    action_ptr = tactic.getNextAction();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(action_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_FALSE(tactic.hasShotAvailable());

    auto intercept_action = std::dynamic_pointer_cast<InterceptBallAction>(action_ptr);
    ASSERT_NE(nullptr, intercept_action);
    ASSERT_TRUE(intercept_action->getRobot().has_value());
    EXPECT_EQ(0, intercept_action->getRobot()->id());
}

TEST(ShootGoalTacticTest, robot_will_intercept_ball_if_shot_is_blocked)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.updateFriendlyTeamState(Team({robot}));
    world = ::TestUtil::setEnemyRobotPositions(world, {Point(1, 0)},
                                               Timestamp::fromSeconds(0));
    BallState ball_state(Point(ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0));
    world.updateBall(Ball(ball_state, Timestamp::fromSeconds(0)));

    ShootGoalTactic tactic = ShootGoalTactic(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        Angle::fromDegrees(1.27), std::nullopt, false, shoot_goal_tactic);
    tactic.updateRobot(robot);
    tactic.updateWorldParams(world);

    auto action_ptr = tactic.getNextAction();

    // Check an intent was returned (the pointer is not null)
    ASSERT_NE(nullptr, action_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_FALSE(tactic.hasShotAvailable());

    auto intercept_action = std::dynamic_pointer_cast<InterceptBallAction>(action_ptr);
    ASSERT_NE(nullptr, intercept_action);
    ASSERT_TRUE(intercept_action->getRobot().has_value());
    EXPECT_EQ(0, intercept_action->getRobot()->id());
}

TEST(ShootGoalTacticTest, robot_will_chip_ball_if_enemy_close_to_stealing_ball)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.updateFriendlyTeamState(Team({robot}));
    world = ::TestUtil::setEnemyRobotPositions(world, {Point(0.25, 0)},
                                               Timestamp::fromSeconds(0));
    BallState ball_state(Point(ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0));
    world.updateBall(Ball(ball_state, Timestamp::fromSeconds(0)));

    ShootGoalTactic tactic = ShootGoalTactic(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        Angle::fromDegrees(1.27), std::nullopt, false, shoot_goal_tactic);
    tactic.updateRobot(robot);
    tactic.updateWorldParams(world);

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
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.updateFriendlyTeamState(Team({robot}));

    BallState ball_state(Point(0.5, 0), Vector(0, 0));
    world.updateBall(Ball(ball_state, Timestamp::fromSeconds(0)));

    ShootGoalTactic tactic = ShootGoalTactic(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        Angle::fromDegrees(1.27), std::nullopt, false, shoot_goal_tactic);

    double cost          = tactic.calculateRobotCost(robot, world);
    double expected_cost = 0.0520833333;
    EXPECT_NEAR(cost, expected_cost, 1e-6);
}

TEST(ShootGoalTacticTest, test_calculate_robot_cost_when_robot_far_from_ball)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.updateFriendlyTeamState(Team({robot}));

    BallState ball_state(Point(3, -2.5), Vector(0, 0));
    world.updateBall(Ball(ball_state, Timestamp::fromSeconds(0)));

    ShootGoalTactic tactic = ShootGoalTactic(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        Angle::fromDegrees(1.27), std::nullopt, false, shoot_goal_tactic);

    double cost          = tactic.calculateRobotCost(robot, world);
    double expected_cost = 0.40678383728;
    EXPECT_NEAR(cost, expected_cost, 1e-6);
}

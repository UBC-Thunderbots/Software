#include "software/ai/hl/stp/tactic/shadow_freekicker_tactic.h"

#include <gtest/gtest.h>

#include "../../../../test_util/test_util.h"
#include "shared/constants.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/util.h"
#include "software/new_geom/line.h"
#include "software/new_geom/util/distance.h"
#include "software/test_util/test_util.h"

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_left_side_with_enemy_robot_at_middle_left_on_field)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(
        world, {Point(0, world.field().friendlyCornerPos().y())},
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the right of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    Line enemy_to_ball_line =
        Line(world.enemyTeam().getAllRobots().at(0).position(), world.ball().position());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle enemy_to_ball_angle =
        (world.ball().position() - world.enemyTeam().getAllRobots().at(0).position())
            .orientation();
    Angle enemy_to_dest_angle = (move_action->getDestination() -
                                 world.enemyTeam().getAllRobots().at(0).position())
                                    .orientation();
    EXPECT_LT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_right_side_with_enemy_robot_at_middle_left_on_field)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(
        world, {Point(0, world.field().friendlyCornerPos().y())},
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the left of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    Line enemy_to_ball_line =
        Line(world.enemyTeam().getAllRobots().at(0).position(), world.ball().position());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle enemy_to_ball_angle =
        (world.ball().position() - world.enemyTeam().getAllRobots().at(0).position())
            .orientation();
    Angle enemy_to_dest_angle = (move_action->getDestination() -
                                 world.enemyTeam().getAllRobots().at(0).position())
                                    .orientation();
    EXPECT_GT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_left_side_with_enemy_robot_at_middle_right_on_field)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(
        world, {Point(0, world.field().friendlyCornerNeg().y())},
        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallPosition(
        world, Point(-0.09, world.field().friendlyCornerNeg().y() + 0.09),
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the right of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    Line enemy_to_ball_line =
        Line(world.enemyTeam().getAllRobots().at(0).position(), world.ball().position());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle enemy_to_ball_angle =
        (world.ball().position() - world.enemyTeam().getAllRobots().at(0).position())
            .orientation();
    Angle enemy_to_dest_angle = (move_action->getDestination() -
                                 world.enemyTeam().getAllRobots().at(0).position())
                                    .orientation();
    EXPECT_LT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_right_side_with_enemy_robot_at_middle_right_on_field)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(
        world, {Point(0, world.field().friendlyCornerNeg().y())},
        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallPosition(
        world, Point(-0.09, world.field().friendlyCornerNeg().y() + 0.09),
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the left of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    Line enemy_to_ball_line =
        Line(world.enemyTeam().getAllRobots().at(0).position(), world.ball().position());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle enemy_to_ball_angle =
        (world.ball().position() - world.enemyTeam().getAllRobots().at(0).position())
            .orientation();
    Angle enemy_to_dest_angle = (move_action->getDestination() -
                                 world.enemyTeam().getAllRobots().at(0).position())
                                    .orientation();
    EXPECT_GT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_left_side_with_enemy_robot_at_top_left_on_field)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(
        world,
        {Point(world.field().friendlyCornerPos().x(),
               world.field().friendlyCornerPos().y())},
        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallPosition(
        world,
        Point(world.field().friendlyCornerPos().x() - 0.09,
              world.field().friendlyCornerPos().y() - 0.09),
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the right of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    Line enemy_to_ball_line =
        Line(world.enemyTeam().getAllRobots().at(0).position(), world.ball().position());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle enemy_to_ball_angle =
        (world.ball().position() - world.enemyTeam().getAllRobots().at(0).position())
            .orientation();
    Angle enemy_to_dest_angle = (move_action->getDestination() -
                                 world.enemyTeam().getAllRobots().at(0).position())
                                    .orientation();
    EXPECT_LT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_right_side_with_enemy_robot_at_top_left_on_field)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(
        world,
        {Point(world.field().friendlyCornerPos().x(),
               world.field().friendlyCornerPos().y())},
        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallPosition(
        world,
        Point(world.field().friendlyCornerPos().x() - 0.09,
              world.field().friendlyCornerPos().y() - 0.09),
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the left of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    Line enemy_to_ball_line =
        Line(world.enemyTeam().getAllRobots().at(0).position(), world.ball().position());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle enemy_to_ball_angle =
        (world.ball().position() - world.enemyTeam().getAllRobots().at(0).position())
            .orientation();
    Angle enemy_to_dest_angle = (move_action->getDestination() -
                                 world.enemyTeam().getAllRobots().at(0).position())
                                    .orientation();
    EXPECT_GT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_left_side_with_enemy_robot_at_top_right_on_field)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(
        world,
        {Point(world.field().friendlyCornerPos().x(),
               world.field().friendlyCornerNeg().y())},
        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallPosition(
        world,
        Point(world.field().friendlyCornerPos().x() - 0.09,
              world.field().friendlyCornerNeg().y() + 0.09),
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the right of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    Line enemy_to_ball_line =
        Line(world.enemyTeam().getAllRobots().at(0).position(), world.ball().position());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle enemy_to_ball_angle =
        (world.ball().position() - world.enemyTeam().getAllRobots().at(0).position())
            .orientation();
    Angle enemy_to_dest_angle = (move_action->getDestination() -
                                 world.enemyTeam().getAllRobots().at(0).position())
                                    .orientation();
    EXPECT_LT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_right_side_with_enemy_robot_at_top_right_on_field)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(
        world,
        {Point(world.field().friendlyCornerPos().x(),
               world.field().friendlyCornerNeg().y())},
        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallPosition(
        world,
        Point(world.field().friendlyCornerPos().x() - 0.09,
              world.field().friendlyCornerNeg().y() + 0.09),
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the left of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    Line enemy_to_ball_line =
        Line(world.enemyTeam().getAllRobots().at(0).position(), world.ball().position());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle enemy_to_ball_angle =
        (world.ball().position() - world.enemyTeam().getAllRobots().at(0).position())
            .orientation();
    Angle enemy_to_dest_angle = (move_action->getDestination() -
                                 world.enemyTeam().getAllRobots().at(0).position())
                                    .orientation();
    EXPECT_GT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_left_side_with_enemy_robot_at_bottom_left_on_field)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(
        world,
        {Point(world.field().friendlyCornerNeg().x(),
               world.field().friendlyCornerPos().y())},
        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallPosition(
        world,
        Point(world.field().friendlyCornerPos().x() + 0.09,
              world.field().friendlyCornerPos().y() - 0.09),
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the right of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    Line enemy_to_ball_line =
        Line(world.enemyTeam().getAllRobots().at(0).position(), world.ball().position());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle enemy_to_ball_angle =
        (world.ball().position() - world.enemyTeam().getAllRobots().at(0).position())
            .orientation();
    Angle enemy_to_dest_angle = (move_action->getDestination() -
                                 world.enemyTeam().getAllRobots().at(0).position())
                                    .orientation();
    EXPECT_LT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_right_side_with_enemy_robot_at_bottom_left_on_field)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(
        world,
        {Point(world.field().friendlyCornerPos().x(),
               world.field().friendlyCornerPos().y())},
        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallPosition(
        world,
        Point(world.field().friendlyCornerPos().x() + 0.09,
              world.field().friendlyCornerPos().y() - 0.09),
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the left of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    Line enemy_to_ball_line =
        Line(world.enemyTeam().getAllRobots().at(0).position(), world.ball().position());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle enemy_to_ball_angle =
        (world.ball().position() - world.enemyTeam().getAllRobots().at(0).position())
            .orientation();
    Angle enemy_to_dest_angle = (move_action->getDestination() -
                                 world.enemyTeam().getAllRobots().at(0).position())
                                    .orientation();
    EXPECT_GT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_left_side_with_enemy_robot_at_botom_right_on_field)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(
        world,
        {Point(world.field().friendlyCornerNeg().x(),
               world.field().friendlyCornerNeg().y())},
        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallPosition(
        world,
        Point(world.field().friendlyCornerNeg().x() + 0.09,
              world.field().friendlyCornerNeg().y() + 0.09),
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the right of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    Line enemy_to_ball_line =
        Line(world.enemyTeam().getAllRobots().at(0).position(), world.ball().position());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle enemy_to_ball_angle =
        (world.ball().position() - world.enemyTeam().getAllRobots().at(0).position())
            .orientation();
    Angle enemy_to_dest_angle = (move_action->getDestination() -
                                 world.enemyTeam().getAllRobots().at(0).position())
                                    .orientation();
    EXPECT_LT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_right_side_with_enemy_robot_at_bottom_right_on_field)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(
        world,
        {Point(world.field().friendlyCornerNeg().x(),
               world.field().friendlyCornerNeg().y())},
        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallPosition(
        world,
        Point(world.field().friendlyCornerNeg().x() + 0.09,
              world.field().friendlyCornerNeg().y() + 0.09),
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the left of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    Line enemy_to_ball_line =
        Line(world.enemyTeam().getAllRobots().at(0).position(), world.ball().position());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle enemy_to_ball_angle =
        (world.ball().position() - world.enemyTeam().getAllRobots().at(0).position())
            .orientation();
    Angle enemy_to_dest_angle = (move_action->getDestination() -
                                 world.enemyTeam().getAllRobots().at(0).position())
                                    .orientation();
    EXPECT_GT(enemy_to_dest_angle, enemy_to_ball_angle);
}


TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_left_side_with_enemy_robot_at_origin_on_field)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0, 0)},
                                                     Timestamp::fromSeconds(0));
    world       = ::Test::TestUtil::setBallPosition(world, Point(0, -0.09),
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the right of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    Line enemy_to_ball_line =
        Line(world.enemyTeam().getAllRobots().at(0).position(), world.ball().position());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle enemy_to_ball_angle =
        (world.ball().position() - world.enemyTeam().getAllRobots().at(0).position())
            .orientation();
    Angle enemy_to_dest_angle = (move_action->getDestination() -
                                 world.enemyTeam().getAllRobots().at(0).position())
                                    .orientation();
    EXPECT_LT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_right_side_with_enemy_robot_at_origin_on_field)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0, 0)},
                                                     Timestamp::fromSeconds(0));
    world       = ::Test::TestUtil::setBallPosition(world, Point(0, -0.09),
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the left of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    Line enemy_to_ball_line =
        Line(world.enemyTeam().getAllRobots().at(0).position(), world.ball().position());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle enemy_to_ball_angle =
        (world.ball().position() - world.enemyTeam().getAllRobots().at(0).position())
            .orientation();
    Angle enemy_to_dest_angle = (move_action->getDestination() -
                                 world.enemyTeam().getAllRobots().at(0).position())
                                    .orientation();
    EXPECT_GT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_left_side_without_robot_in_enemy_team)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setBallPosition(
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the left of the line between the friendly net and
    // the ball (from the POV of the friendly net)
    Line enemy_to_ball_line = Line(world.ball().position(), world.field().friendlyGoal());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle goal_to_ball_angle =
        (world.ball().position() - world.field().friendlyGoal()).orientation();
    Angle goal_to_dest_angle =
        (move_action->getDestination() - world.field().friendlyGoal()).orientation();
    EXPECT_GT(goal_to_dest_angle, goal_to_ball_angle);
}

TEST(ShadowFreekickerTacticTest,
     test_shadow_free_kicker_right_side_without_robot_in_enemy_team)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setBallPosition(
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

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the right of the line between the friendly net and
    // the ball (from the POV of the friendly net)
    Line enemy_to_ball_line = Line(world.ball().position(), world.field().friendlyGoal());
    EXPECT_NEAR(distance(enemy_to_ball_line, move_action->getDestination()), 0.09, 0.01);
    Angle goal_to_ball_angle =
        (world.ball().position() - world.field().friendlyGoal()).orientation();
    Angle goal_to_dest_angle =
        (move_action->getDestination() - world.field().friendlyGoal()).orientation();
    EXPECT_LT(goal_to_dest_angle, goal_to_ball_angle);
}
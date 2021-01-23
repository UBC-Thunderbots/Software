#include "software/ai/hl/stp/tactic/penalty_kick_tactic.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(PenaltyKickTacticTest, no_enemy_goalie)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().penaltyEnemy(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Vector behind_ball_direction =
        -(world.ball().position() - world.field().enemyGoalCenter()).normalize();
    Point behind_ball = world.field().penaltyEnemy() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PenaltyKickTactic tactic =
        PenaltyKickTactic(world.ball(), world.field(), std::nullopt, false);

    EXPECT_TRUE(tactic.evaluatePenaltyShot());
}

TEST(PenaltyKickTacticTest, enemy_goalie_offset_left_no_viable_shot)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().penaltyEnemy(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), 0.2);
    Vector behind_ball_direction =
        -(world.ball().position() - world.field().enemyGoalCenter()).normalize();
    Point behind_ball = world.ball().position() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, 0), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world.updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world.updateFriendlyTeamState(friendly);

    PenaltyKickTactic tactic = PenaltyKickTactic(
        world.ball(), world.field(), std::optional<Robot>{enemy_goalie}, false);
    tactic.updateRobot(shooter);

    EXPECT_FALSE(tactic.evaluatePenaltyShot());
}

TEST(PenaltyKickTacticTest, enemy_goalie_offset_right_no_viable_shot)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().penaltyEnemy(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), -0.2);
    Vector behind_ball_direction =
        -(world.ball().position() - enemy_goalie_pos).normalize();
    Point behind_ball = world.ball().position() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, +0.2), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world.updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world.updateFriendlyTeamState(friendly);

    PenaltyKickTactic tactic = PenaltyKickTactic(
        world.ball(), world.field(), std::optional<Robot>{enemy_goalie}, false);
    tactic.updateRobot(shooter);

    EXPECT_FALSE(tactic.evaluatePenaltyShot());
}

TEST(PenaltyKickTacticTest, enemy_goalie_right_viable_shot_left)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, Point(4, 0),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), +0.2);
    Vector behind_ball_direction =
        -(world.ball().position() - world.field().enemyGoalpostNeg()).normalize();
    Point behind_ball = world.ball().position() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, +0.2), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world.updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world.updateFriendlyTeamState(friendly);

    PenaltyKickTactic tactic = PenaltyKickTactic(
        world.ball(), world.field(), std::optional<Robot>{enemy_goalie}, false);
    tactic.updateRobot(shooter);

    EXPECT_TRUE(tactic.evaluatePenaltyShot());
}

TEST(PenaltyKickTacticTest, enemy_goalie_left_viable_shot_right)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, Point(4, 0),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), -0.2);
    Vector behind_ball_direction =
        -(world.ball().position() - world.field().enemyGoalpostPos()).normalize();
    Point behind_ball = world.ball().position() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, +0.2), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world.updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world.updateFriendlyTeamState(friendly);

    PenaltyKickTactic tactic = PenaltyKickTactic(
        world.ball(), world.field(), std::optional<Robot>{enemy_goalie}, false);
    tactic.updateRobot(shooter);

    EXPECT_TRUE(tactic.evaluatePenaltyShot());
}

TEST(PenaltyKickTacticTest, no_enemy_goalie_shot_position)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().penaltyEnemy(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    PenaltyKickTactic tactic =
        PenaltyKickTactic(world.ball(), world.field(), std::nullopt, false);

    EXPECT_EQ(tactic.evaluateNextShotPosition(), world.field().enemyGoalCenter());
}

TEST(PenaltyKickTacticTest, enemy_goalie_left_shot_right)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().penaltyEnemy(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), 0.2);
    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, 0), Angle::half(),
                               AngularVelocity::zero(),
							   Timestamp::fromSeconds(0));
    PenaltyKickTactic tactic = PenaltyKickTactic(
        world.ball(), world.field(), std::optional<Robot>{enemy_goalie}, false);

	Point shot_position = tactic.evaluateNextShotPosition();
    EXPECT_LE(shot_position.y(), 0);
	EXPECT_EQ(shot_position.x(), world.field().enemyGoalCenter().x());
}

TEST(PenaltyKickTacticTest, enemy_goalie_right_shot_left)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().penaltyEnemy(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), -0.2);
    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, 0), Angle::half(),
                               AngularVelocity::zero(),
							   Timestamp::fromSeconds(0));
    PenaltyKickTactic tactic = PenaltyKickTactic(
        world.ball(), world.field(), std::optional<Robot>{enemy_goalie}, false);

	Point shot_position = tactic.evaluateNextShotPosition();
    EXPECT_GE(shot_position.y(), 0);
	EXPECT_EQ(shot_position.x(), world.field().enemyGoalCenter().x());
}

#include "software/ai/hl/stp/tactic/defense_shadow_enemy_tactic.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/intent/move_intent.h"
#include "software/test_util/test_util.h"

TEST(DefenseShadowEnemyTacticTest, test_shadower_blocks_net_when_enemy_cannot_pass)
{
    Robot enemy_robot(1, Point(0, 0), Vector(0, 0), Angle::zero(),
                      AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Evaluation::EnemyThreat enemy_threat{enemy_robot,  false, Angle::zero(), std::nullopt,
                                         std::nullopt, 0,     std::nullopt};
    Field field        = ::Test::TestUtil::createSSLDivBField();
    Team enemy_team    = Team(Duration::fromSeconds(1), {enemy_robot});
    Team friendly_team = Team(Duration::fromSeconds(1), {friendly_robot});
    Ball ball(Point(1, 1), Vector(0, 0), Timestamp::fromSeconds(0));

    DefenseShadowEnemyTactic tactic =
        DefenseShadowEnemyTactic(field, friendly_team, enemy_team, ball, true, 0.5, true);
    tactic.updateRobot(friendly_robot);
    tactic.updateWorldParams(field, friendly_team, enemy_team, ball);
    tactic.updateControlParams(enemy_threat);

    auto intent_ptr = tactic.getNextIntent();

    ASSERT_TRUE(intent_ptr);

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(move_intent.getDestination().isClose(Point(-0.5, 0), 0.01));
        EXPECT_LT(move_intent.getFinalAngle().minDiff(Angle::zero()),
                  Angle::fromDegrees(1));
        EXPECT_TRUE(move_intent.getAutoKickType() == AutokickType::AUTOCHIP);
    }
    catch (std::bad_cast &)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the ShootGoalTactic!";
    }
}

TEST(DefenseShadowEnemyTacticTest,
     test_shadower_steals_ball_if_the_enemy_has_possession_and_ball_is_moving_slow_enough)
{
    Robot enemy_robot(1, Point(0, 0), Vector(0, 0), Angle::half(),
                      AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot(0, Point(-1, -1), Vector(0, 0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Evaluation::EnemyThreat enemy_threat{enemy_robot,  false, Angle::zero(), std::nullopt,
                                         std::nullopt, 0,     std::nullopt};
    Field field        = ::Test::TestUtil::createSSLDivBField();
    Team enemy_team    = Team(Duration::fromSeconds(1), {enemy_robot});
    Team friendly_team = Team(Duration::fromSeconds(1), {friendly_robot});
    Ball ball(Point(-ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0),
              Timestamp::fromSeconds(0));

    DefenseShadowEnemyTactic tactic =
        DefenseShadowEnemyTactic(field, friendly_team, enemy_team, ball, true, 0.5, true);
    tactic.updateRobot(friendly_robot);
    tactic.updateWorldParams(field, friendly_team, enemy_team, ball);
    tactic.updateControlParams(enemy_threat);

    auto intent_ptr = tactic.getNextIntent();

    ASSERT_TRUE(intent_ptr);

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(move_intent.getDestination().isClose(ball.position(), 0.01));
        EXPECT_LT(move_intent.getFinalAngle().minDiff(
                      (enemy_robot.position() - field.friendlyGoal()).orientation()),
                  Angle::fromDegrees(1));
        EXPECT_TRUE(move_intent.getAutoKickType() == AUTOCHIP);
    }
    catch (std::bad_cast &)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the ShootGoalTactic!";
    }
}


TEST(
    DefenseShadowEnemyTacticTest,
    test_shadower_does_not_steal_ball_if_the_enemy_has_possession_but_the_ball_is_moving_quickly)
{
    Robot enemy_robot(1, Point(0, 0), Vector(0, 0), Angle::half(),
                      AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot(0, Point(-1, -1), Vector(0, 0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Evaluation::EnemyThreat enemy_threat{enemy_robot,  false, Angle::zero(), std::nullopt,
                                         std::nullopt, 0,     std::nullopt};
    Field field        = ::Test::TestUtil::createSSLDivBField();
    Team enemy_team    = Team(Duration::fromSeconds(1), {enemy_robot});
    Team friendly_team = Team(Duration::fromSeconds(1), {friendly_robot});
    Ball ball(Point(-ROBOT_MAX_RADIUS_METERS, 0), Vector(4, 3),
              Timestamp::fromSeconds(0));

    DefenseShadowEnemyTactic tactic =
        DefenseShadowEnemyTactic(field, friendly_team, enemy_team, ball, true, 0.5, true);
    tactic.updateRobot(friendly_robot);
    tactic.updateWorldParams(field, friendly_team, enemy_team, ball);
    tactic.updateControlParams(enemy_threat);

    auto intent_ptr = tactic.getNextIntent();

    ASSERT_TRUE(intent_ptr);

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(move_intent.getDestination().isClose(Point(-0.5, 0), 0.01));
        EXPECT_LT(move_intent.getFinalAngle().minDiff(
                      (enemy_robot.position() - friendly_robot.position()).orientation()),
                  Angle::fromDegrees(1));
        EXPECT_TRUE(move_intent.getAutoKickType() == AUTOCHIP);
    }
    catch (std::bad_cast &)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the ShootGoalTactic!";
    }
}

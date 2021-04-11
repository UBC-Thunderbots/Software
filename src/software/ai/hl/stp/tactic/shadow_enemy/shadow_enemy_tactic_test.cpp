// #include "software/ai/hl/stp/tactic/shadow_enemy_tactic.h"

// #include <gtest/gtest.h>

// #include "shared/constants.h"
// #include "software/ai/evaluation/enemy_threat.h"
// #include "software/ai/hl/stp/action/move_action.h"
// #include "software/test_util/test_util.h"

// TEST(ShadowEnemyTacticTest, test_shadower_blocks_net_when_enemy_cannot_pass)
// {
//     Robot enemy_robot(1, Point(0, 0), Vector(0, 0), Angle::zero(),
//                       AngularVelocity::zero(), Timestamp::fromSeconds(0));
//     Robot friendly_robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
//                          AngularVelocity::zero(), Timestamp::fromSeconds(0));

//     EnemyThreat enemy_threat{enemy_robot,  false, Angle::zero(), std::nullopt,
//                              std::nullopt, 0,     std::nullopt};
//     Field field        = Field::createSSLDivisionBField();
//     Team enemy_team    = Team({enemy_robot}, Duration::fromSeconds(1));
//     Team friendly_team = Team({friendly_robot}, Duration::fromSeconds(1));
//     Ball ball(Point(1, 1), Vector(0, 0), Timestamp::fromSeconds(0));
//     World world = World(field, ball, friendly_team, enemy_team);

//     ShadowEnemyTactic tactic =
//         ShadowEnemyTactic(field, friendly_team, enemy_team, true, ball, 0.5, false,
//         true);
//     tactic.updateRobot(friendly_robot);
//     tactic.updateWorldParams(world);
//     tactic.updateControlParams(enemy_threat, 0.5);

//     auto action_ptr = tactic.getNextAction();

//     ASSERT_TRUE(action_ptr);

//     auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
//     ASSERT_NE(nullptr, move_action);
//     EXPECT_TRUE(TestUtil::equalWithinTolerance(move_action->getDestination(),
//                                                Point(-0.5, 0), 0.01));
//     EXPECT_LT(move_action->getFinalOrientation().minDiff(Angle::zero()),
//               Angle::fromDegrees(1));
//     EXPECT_EQ(move_action->getAutoChipOrKick().auto_chip_kick_mode,
//               AutoChipOrKickMode::OFF);
// }

// TEST(ShadowEnemyTacticTest, test_shadower_blocks_pass_when_enemy_can_pass)
// {
//     Robot enemy_robot(1, Point(0, 0), Vector(0, 0), Angle::zero(),
//                       AngularVelocity::zero(), Timestamp::fromSeconds(0));
//     Robot enemy_robot_2(2, Point(0, 2), Vector(0, 0), Angle::zero(),
//                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
//     Robot friendly_robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
//                          AngularVelocity::zero(), Timestamp::fromSeconds(0));

//     EnemyThreat enemy_threat{enemy_robot,
//                              false,
//                              Angle::zero(),
//                              std::nullopt,
//                              std::nullopt,
//                              1,
//                              std::make_optional(enemy_robot_2)};
//     Field field        = Field::createSSLDivisionBField();
//     Team enemy_team    = Team({enemy_robot, enemy_robot_2}, Duration::fromSeconds(1));
//     Team friendly_team = Team({friendly_robot}, Duration::fromSeconds(1));
//     Ball ball(Point(1, 1), Vector(0, 0), Timestamp::fromSeconds(0));
//     World world = World(field, ball, friendly_team, enemy_team);

//     ShadowEnemyTactic tactic =
//         ShadowEnemyTactic(field, friendly_team, enemy_team, true, ball, 0, true, true);
//     tactic.updateRobot(friendly_robot);
//     tactic.updateWorldParams(world);
//     tactic.updateControlParams(enemy_threat, 0.2);

//     auto action_ptr = tactic.getNextAction();

//     ASSERT_TRUE(action_ptr);

//     auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
//     ASSERT_NE(nullptr, move_action);
//     EXPECT_TRUE(TestUtil::equalWithinTolerance(move_action->getDestination(),
//                                                Point(0, 0.2), 0.01));
//     EXPECT_LT(move_action->getFinalOrientation().minDiff(Angle::quarter()),
//               Angle::fromDegrees(1));
//     EXPECT_EQ(move_action->getAutoChipOrKick().auto_chip_kick_mode,
//               AutoChipOrKickMode::OFF);
// }


// TEST(ShadowEnemyTacticTest,
//      test_shadower_steals_ball_if_the_enemy_has_possession_and_ball_is_moving_slow_enough)
// {
//     Robot enemy_robot(1, Point(0, 0), Vector(0, 0), Angle::half(),
//                       AngularVelocity::zero(), Timestamp::fromSeconds(0));
//     Robot friendly_robot(0, Point(-1, -1), Vector(0, 0), Angle::zero(),
//                          AngularVelocity::zero(), Timestamp::fromSeconds(0));

//     EnemyThreat enemy_threat{enemy_robot,  false, Angle::zero(), std::nullopt,
//                              std::nullopt, 0,     std::nullopt};
//     Field field        = Field::createSSLDivisionBField();
//     Team enemy_team    = Team({enemy_robot}, Duration::fromSeconds(1));
//     Team friendly_team = Team({friendly_robot}, Duration::fromSeconds(1));
//     Ball ball(Point(-ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0),
//               Timestamp::fromSeconds(0));
//     World world = World(field, ball, friendly_team, enemy_team);

//     ShadowEnemyTactic tactic =
//         ShadowEnemyTactic(field, friendly_team, enemy_team, true, ball, 0, true, true);
//     tactic.updateRobot(friendly_robot);
//     tactic.updateWorldParams(world);
//     tactic.updateControlParams(enemy_threat, 0.5);

//     auto action_ptr = tactic.getNextAction();

//     ASSERT_TRUE(action_ptr);

//     auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
//     ASSERT_NE(nullptr, move_action);
//     EXPECT_TRUE(TestUtil::equalWithinTolerance(move_action->getDestination(),
//                                                ball.position(), 0.01));
//     EXPECT_LT(move_action->getFinalOrientation().minDiff(
//                   (ball.position() - friendly_robot.position()).orientation()),
//               Angle::fromDegrees(1));
//     ASSERT_EQ(move_action->getAutoChipOrKick().auto_chip_kick_mode,
//               AutoChipOrKickMode::AUTOCHIP);
//     EXPECT_EQ(move_action->getAutoChipOrKick().autochip_distance_m,
//               ShadowEnemyTactic::YEET_CHIP_DISTANCE_METERS);
// }

// TEST(
//     ShadowEnemyTacticTest,
//     test_shadower_does_not_steal_ball_if_the_enemy_has_possession_but_the_ball_is_moving_quickly)
// {
//     Robot enemy_robot(1, Point(0, 0), Vector(0, 0), Angle::half(),
//                       AngularVelocity::zero(), Timestamp::fromSeconds(0));
//     Robot friendly_robot(0, Point(-1, -1), Vector(0, 0), Angle::zero(),
//                          AngularVelocity::zero(), Timestamp::fromSeconds(0));

//     EnemyThreat enemy_threat{enemy_robot,  false, Angle::zero(), std::nullopt,
//                              std::nullopt, 0,     std::nullopt};
//     Field field        = Field::createSSLDivisionBField();
//     Team enemy_team    = Team({enemy_robot}, Duration::fromSeconds(1));
//     Team friendly_team = Team({friendly_robot}, Duration::fromSeconds(1));
//     Ball ball(Point(-ROBOT_MAX_RADIUS_METERS, 0), Vector(4, 3),
//               Timestamp::fromSeconds(0));
//     World world = World(field, ball, friendly_team, enemy_team);

//     ShadowEnemyTactic tactic =
//         ShadowEnemyTactic(field, friendly_team, enemy_team, true, ball, 0.5, true,
//         true);
//     tactic.updateRobot(friendly_robot);
//     tactic.updateWorldParams(world);
//     tactic.updateControlParams(enemy_threat, 0.5);

//     auto action_ptr = tactic.getNextAction();

//     ASSERT_TRUE(action_ptr);

//     auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
//     ASSERT_NE(nullptr, move_action);
//     EXPECT_TRUE(TestUtil::equalWithinTolerance(move_action->getDestination(),
//                                                Point(-0.5, 0), 0.01));
//     EXPECT_LT(move_action->getFinalOrientation().minDiff(Angle::zero()),
//               Angle::fromDegrees(1));
//     EXPECT_EQ(move_action->getAutoChipOrKick().auto_chip_kick_mode,
//               AutoChipOrKickMode::OFF);
// }

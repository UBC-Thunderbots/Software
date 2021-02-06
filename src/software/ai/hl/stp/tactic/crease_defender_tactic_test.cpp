#include "software/ai/hl/stp/tactic/crease_defender_tactic.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/test_util/test_util.h"

TEST(CreaseDefenderTacticTest, single_defender_blocks_shot_without_goalie)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    ::TestUtil::setEnemyRobotPositions(world, {Point(0.09, 0)},
                                       Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.updateFriendlyTeamState(Team({friendly_robot}));

    CreaseDefenderTactic tactic =
        CreaseDefenderTactic(world.field(), world.ball(), world.friendlyTeam(),
                             world.enemyTeam(), CreaseDefenderTactic::LEFT);
    tactic.updateRobot(friendly_robot);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        move_action->getDestination(),
        Point(world.field().friendlyDefenseArea().posXPosYCorner().x() +
                  ROBOT_MAX_RADIUS_METERS,
              0.0),
        0.05));
}

TEST(CreaseDefenderTacticTest, single_defender_blocks_shot_with_goalie_left_side)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    ::TestUtil::setEnemyRobotPositions(world, {Point(0.09, 0)},
                                       Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_goalie =
        Robot(1, world.field().friendlyGoalCenter(), Vector(0, 0), Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team new_team({friendly_robot, friendly_goalie});
    new_team.assignGoalie(1);
    world.updateFriendlyTeamState(new_team);

    CreaseDefenderTactic tactic =
        CreaseDefenderTactic(world.field(), world.ball(), world.friendlyTeam(),
                             world.enemyTeam(), CreaseDefenderTactic::LEFT);
    tactic.updateWorldParams(world);
    EXPECT_EQ(tactic.getBall().position(), Point(0, 0));

    tactic.updateRobot(friendly_robot);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    // The robot's position should be one full robot diameter to the left,
    // perpendicular to the shot vector, so that the goalie is allowed to block the
    // shot in the middle and the crease defender isn't overlapping with the goalie
    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        move_action->getDestination(),
        Point(world.field().friendlyDefenseArea().posXPosYCorner().x() +
                  ROBOT_MAX_RADIUS_METERS,
              2 * ROBOT_MAX_RADIUS_METERS),
        0.05));
}

TEST(CreaseDefenderTacticTest, single_defender_blocks_shot_with_goalie_right_side)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    ::TestUtil::setEnemyRobotPositions(world, {Point(0.09, 0)},
                                       Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_goalie =
        Robot(1, world.field().friendlyGoalCenter(), Vector(0, 0), Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Team new_team({friendly_robot, friendly_goalie});
    new_team.assignGoalie(1);
    world.updateFriendlyTeamState(new_team);

    CreaseDefenderTactic tactic =
        CreaseDefenderTactic(world.field(), world.ball(), world.friendlyTeam(),
                             world.enemyTeam(), CreaseDefenderTactic::RIGHT);
    tactic.updateRobot(friendly_robot);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    // The robot's position should be one full robot diameter to the right,
    // perpendicular to the shot vector, so that the goalie is allowed to block the
    // shot in the middle and the crease defender isn't overlapping with the goalie
    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        move_action->getDestination(),
        Point(world.field().friendlyDefenseArea().posXPosYCorner().x() +
                  ROBOT_MAX_RADIUS_METERS,
              -2 * ROBOT_MAX_RADIUS_METERS),
        0.05));
}

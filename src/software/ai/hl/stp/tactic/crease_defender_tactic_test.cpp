#include "software/ai/hl/stp/tactic/crease_defender_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/intent/move_intent.h"
#include "software/test_util/test_util.h"
#include "shared/constants.h"

TEST(BlockShotPathTacticTest, single_defender_blocks_shot_without_goalie)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.09, 0)}, Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot});

    CreaseDefenderTactic tactic = CreaseDefenderTactic(world.field(), world.ball(), world.friendlyTeam(), world.enemyTeam(), CreaseDefenderTactic::LEFT);
    tactic.updateRobot(friendly_robot);
    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);

    try {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(move_intent.getDestination().isClose(Point(world.field().friendlyDefenseArea().posXPosYCorner().x() + ROBOT_MAX_RADIUS_METERS, 0.0), 0.05));
    }
    catch (...) {
        ADD_FAILURE() << "MoveIntent was not returned by the ShootGoalTactic!";
    }
}

TEST(BlockShotPathTacticTest, single_defender_blocks_shot_with_goalie_left_side)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.09, 0)}, Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_goalie = Robot(1, world.field().friendlyGoal(), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot, friendly_goalie});
    world.mutableFriendlyTeam().assignGoalie(1);

    CreaseDefenderTactic tactic = CreaseDefenderTactic(world.field(), world.ball(), world.friendlyTeam(), world.enemyTeam(), CreaseDefenderTactic::LEFT);
    tactic.updateRobot(friendly_robot);
    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);

    try {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(move_intent.getDestination().isClose(Point(world.field().friendlyDefenseArea().posXPosYCorner().x() + ROBOT_MAX_RADIUS_METERS, 0.18), 0.05));
    }
    catch (...) {
        ADD_FAILURE() << "MoveIntent was not returned by the ShootGoalTactic!";
    }
}

TEST(BlockShotPathTacticTest, single_defender_blocks_shot_with_goalie_right_side)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.09, 0)}, Timestamp::fromSeconds(0));

    Robot friendly_robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_goalie = Robot(1, world.field().friendlyGoal(), Vector(0, 0), Angle::zero(),
                                  AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot, friendly_goalie});
    world.mutableFriendlyTeam().assignGoalie(1);

    CreaseDefenderTactic tactic = CreaseDefenderTactic(world.field(), world.ball(), world.friendlyTeam(), world.enemyTeam(), CreaseDefenderTactic::RIGHT);
    tactic.updateRobot(friendly_robot);
    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);

    try {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(move_intent.getDestination().isClose(Point(world.field().friendlyDefenseArea().posXPosYCorner().x() + ROBOT_MAX_RADIUS_METERS, -0.18), 0.05));
    }
    catch (...) {
        ADD_FAILURE() << "MoveIntent was not returned by the ShootGoalTactic!";
    }
}

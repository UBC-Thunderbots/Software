#include "ai/hl/stp/tactic/block_shot_path_tactic.h"

#include <gtest/gtest.h>

#include "ai/intent/move_intent.h"
#include "geom/util.h"
#include "test/test_util/test_util.h"

TEST(BlockShotPathTacticTest, shot_starts_close_to_net)
{
    Field field = ::Test::TestUtil::createSSLDivBField();

    Robot friendly_robot = Robot(0, Point(1, 0.5), Vector(), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));

    BlockShotPathTactic tactic = BlockShotPathTactic(field);
    tactic.updateRobot(friendly_robot);
    // Shoot from 2m in front of the goal
    Point shot_origin = field.friendlyGoal() + Point(2, 0);
    tactic.updateParams(shot_origin);
    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);

    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
    EXPECT_EQ(0, move_intent.getRobotId());
    // Check the robot is moving somewhere on the line segment between the shot origin
    // and the goal
    EXPECT_LE(
        dist(Segment(shot_origin, field.friendlyGoal()), move_intent.getDestination()),
        0.1);
    // Make sure the robot is facing the shot
    EXPECT_EQ(Angle::zero(), move_intent.getFinalAngle());
    EXPECT_EQ(0.0, move_intent.getFinalSpeed());
}

TEST(BlockShotPathTacticTest, shot_starts_far_from_the_net)
{
    Field field = ::Test::TestUtil::createSSLDivBField();

    Robot friendly_robot = Robot(0, Point(1, 0.5), Vector(), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot enemy_robot = Robot(0, field.enemyCornerNeg(), Vector(), Angle::zero(),
                              AngularVelocity::zero(), Timestamp::fromSeconds(0));

    BlockShotPathTactic tactic = BlockShotPathTactic(field);
    tactic.updateRobot(friendly_robot);
    tactic.updateParams(enemy_robot);
    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);

    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
    EXPECT_EQ(0, move_intent.getRobotId());
    // Check the robot is moving somewhere on the line segment between the shot origin
    // and the goal
    EXPECT_LE(dist(Segment(enemy_robot.position(), field.friendlyGoal()),
                   move_intent.getDestination()),
              0.1);
    // Make sure the robot is facing the shot
    EXPECT_NEAR((enemy_robot.position() - field.friendlyGoal()).orientation().toRadians(),
                move_intent.getFinalAngle().toRadians(), 0.001);
    EXPECT_EQ(0.0, move_intent.getFinalSpeed());
}

TEST(BlockShotPathTacticTest, test_calculate_robot_cost)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    Robot robot = Robot(0, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    BlockShotPathTactic tactic = BlockShotPathTactic(world.field());
    tactic.updateParams(Point());

    // The robot is a little over 2 meters away from where we expect it to block, so the
    // cost should be relatively low
    double cost = tactic.calculateRobotCost(robot, world);

    EXPECT_NEAR(0.2, cost, 0.1);
}

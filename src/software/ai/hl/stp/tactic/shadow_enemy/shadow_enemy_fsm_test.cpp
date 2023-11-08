#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(ShadowEnemyFSMTest, test_findBlockPassPoint)
{
    Point ball_position    = Point(0, 2);
    Robot shadowee         = Robot(0, Point(0, -2), Vector(0, 0), Angle::half(),
                           AngularVelocity::zero(), Timestamp::fromSeconds(0));
    double shadow_distance = 2;
    Point block_pass_point =
        ShadowEnemyFSM::findBlockPassPoint(ball_position, shadowee, shadow_distance);
    EXPECT_EQ(block_pass_point, Point(0, 0));

    ball_position = Point(0, 2);
    shadowee = Robot(0, Point(2, 0), Vector(0, 0), Angle::half(), AngularVelocity::zero(),
                     Timestamp::fromSeconds(0));
    shadow_distance = 1;
    block_pass_point =
        ShadowEnemyFSM::findBlockPassPoint(ball_position, shadowee, shadow_distance);
    EXPECT_EQ(block_pass_point, Point(2 - 1 / std::sqrt(2), 1 / std::sqrt(2)));


    ball_position   = Point(2, 0);
    shadowee        = Robot(0, Point(-2, 0), Vector(0, 0), Angle::half(),
                     AngularVelocity::zero(), Timestamp::fromSeconds(0));
    shadow_distance = 0.25;
    block_pass_point =
        ShadowEnemyFSM::findBlockPassPoint(ball_position, shadowee, shadow_distance);
    EXPECT_EQ(block_pass_point, Point(-1.75, 0));
}

TEST(ShadowEnemyFSMTest, test_findBlockShotPoint)
{
    Robot shadower    = Robot(0, Point(0, 1), Vector(0, 0), Angle::half(),
                           AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot shadowee    = Robot(0, Point(2, 0), Vector(0, 0), Angle::half(),
                           AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Field field       = Field::createSSLDivisionBField();
    Team friendlyTeam = Team(Duration::fromSeconds(1));
    friendlyTeam.updateRobots({shadower});
    Team enemyTeam = Team(Duration::fromSeconds(1));
    enemyTeam.updateRobots({shadowee});
    double shadow_distance = 2;


    Point block_shot_point = ShadowEnemyFSM::findBlockShotPoint(
        shadower, field, friendlyTeam, enemyTeam, shadowee, shadow_distance);
    EXPECT_EQ(block_shot_point, Point(0, 0));

    shadower = Robot(0, Point(0, 1), Vector(0, 0), Angle::half(), AngularVelocity::zero(),
                     Timestamp::fromSeconds(0));
    shadowee = Robot(0, Point(1, 2), Vector(0, 0), Angle::half(), AngularVelocity::zero(),
                     Timestamp::fromSeconds(0));
    friendlyTeam.updateRobots({shadower});
    enemyTeam.updateRobots({shadowee});

    block_shot_point = ShadowEnemyFSM::findBlockShotPoint(
        shadower, field, friendlyTeam, enemyTeam, shadowee, shadow_distance);
    auto enemy_shot = field.friendlyGoalCenter() - shadowee.position();
    EXPECT_EQ(block_shot_point,
              shadowee.position() + enemy_shot.normalize(shadow_distance));
}


TEST(ShadowEnemyFSMTest, test_transitions)
{
    Robot enemy    = ::TestUtil::createRobotAtPos(Point(0, 2));
    Robot shadowee = ::TestUtil::createRobotAtPos(Point(0, -2));
    Robot shadower = ::TestUtil::createRobotAtPos(Point(-2, 0));
    World world    = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 2), Timestamp::fromSeconds(0));
    EnemyThreat enemy_threat{shadowee,     false, Angle::zero(), std::nullopt,
                             std::nullopt, 1,     enemy};
    FSM<ShadowEnemyFSM> fsm;

    // Start in MoveFSM
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM>));

    // Enemy has the ball but not the shadowee
    // Robot should be trying to block possible pass to the shadowee
    fsm.process_event(ShadowEnemyFSM::Update(
        {enemy_threat, 0.5},
        TacticUpdate(
                shadower, world, [](std::unique_ptr<TbotsProto::Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ShadowEnemyFSM::BlockPassState>));

    // Shadowee now has the ball, our robot should move to block the shot
    // Robot should be trying to block possible shot on our net
    enemy_threat.has_ball = true;
    world = ::TestUtil::setBallPosition(world, Point(0, -2), Timestamp::fromSeconds(0));
    fsm.process_event(ShadowEnemyFSM::Update(
        {enemy_threat, 0.5},
        TacticUpdate(
                shadower, world, [](std::unique_ptr<TbotsProto::Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM>));

    // Shadowee now has the ball
    // Robot should still be in block shot state if not in correct
    // shot block position
    fsm.process_event(ShadowEnemyFSM::Update(
        {enemy_threat, 0.5},
        TacticUpdate(
                shadower, world, [](std::unique_ptr<TbotsProto::Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM>));

    // Shadowee still has possession of the ball and robot has arrived at block shot
    // position Robot should try to steal and chip the ball
    Point position_to_block = ShadowEnemyFSM::findBlockShotPoint(
        shadower, world.field(), world.friendlyTeam(), world.enemyTeam(), shadowee, 0.5);
    shadower.updateState(
        RobotState(position_to_block, Vector(),
                   (world.ball().position() - position_to_block).orientation(),
                   AngularVelocity::zero()),
        Timestamp::fromSeconds(0));
    fsm.process_event(ShadowEnemyFSM::Update(
        {enemy_threat, 0.5},
        TacticUpdate(
                shadower, world, [](std::unique_ptr<TbotsProto::Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ShadowEnemyFSM::StealAndChipState>));

    // Shadowee still has possession of the ball
    // Robot should continue to try and steal and chip the ball
    fsm.process_event(ShadowEnemyFSM::Update(
        {enemy_threat, 0.5},
        TacticUpdate(
                shadower, world, [](std::unique_ptr<TbotsProto::Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ShadowEnemyFSM::StealAndChipState>));

    // Either the ball has been stolen and chipped by our robot or the
    // enemy threat has kicked the ball
    // Tactic is done
    enemy_threat.has_ball = false;
    world = ::TestUtil::setBallPosition(world, Point(0, 2), Timestamp::fromSeconds(0));
    fsm.process_event(ShadowEnemyFSM::Update(
        {enemy_threat, 0.5},
        TacticUpdate(
                shadower, world, [](std::unique_ptr<TbotsProto::Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    // Enemy has the ball but not the shadowee (same as first transition)
    // Robot should be trying to block possible pass to the shadowee
    fsm.process_event(ShadowEnemyFSM::Update(
        {enemy_threat, 0.5},
        TacticUpdate(
                shadower, world, [](std::unique_ptr<TbotsProto::Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ShadowEnemyFSM::BlockPassState>));
}

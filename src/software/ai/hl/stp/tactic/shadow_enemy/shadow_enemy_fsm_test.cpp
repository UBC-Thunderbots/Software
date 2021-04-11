#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(ShadowEnemyFSMTest, test_transitions)
{
    Robot enemy    = ::TestUtil::createRobotAtPos(Point(0, 2));
    Robot shadowee = ::TestUtil::createRobotAtPos(Point(0, -2));
    Robot shadower = ::TestUtil::createRobotAtPos(Point(-2, 0));
    World world    = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(0, 2), Timestamp::fromSeconds(123));
    EnemyThreat enemy_threat{shadowee,     false, Angle::zero(), std::nullopt,
                             std::nullopt, 1,     enemy};
    FSM<ShadowEnemyFSM> fsm;

    // Start in BlockState
    EXPECT_TRUE(fsm.is(boost::sml::state<ShadowEnemyFSM::BlockState>));

    // Enemy has the ball but not the shadowee
    // Robot should be trying to block possible pass to the shadowee
    fsm.process_event(ShadowEnemyFSM::Update(
        {enemy_threat, 0.5, 0.5},
        TacticUpdate(shadower, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ShadowEnemyFSM::BlockState>));

    // Shadowee now has the ball, but our robot should still remain in the block state
    // Robot should be trying to block possible shot on our net
    enemy_threat.has_ball = true;
    world = ::TestUtil::setBallPosition(world, Point(0, -2), Timestamp::fromSeconds(124));
    fsm.process_event(ShadowEnemyFSM::Update(
        {enemy_threat, 0.5, 0.5},
        TacticUpdate(shadower, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ShadowEnemyFSM::BlockState>));


    // Shadowee now has the ball, but our robot is within steal_and_chip distance
    // Robot should try to steal and chip the ball
    fsm.process_event(ShadowEnemyFSM::Update(
        {enemy_threat, 0.5, 10},
        TacticUpdate(shadower, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ShadowEnemyFSM::StealAndChipState>));

    // Shadowee still has possession of the ball
    // Robot should continue to try to steal and chip the ball

    fsm.process_event(ShadowEnemyFSM::Update(
        {enemy_threat, 0.5, 10},
        TacticUpdate(shadower, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ShadowEnemyFSM::StealAndChipState>));

    // Either the ball has been stolen and chipped by our robot or the
    // enemy threat has kicked the ball
    // Tactic is done
    enemy_threat.has_ball = false;
    world = ::TestUtil::setBallPosition(world, Point(0, 2), Timestamp::fromSeconds(125));
    fsm.process_event(ShadowEnemyFSM::Update(
        {enemy_threat, 0.5, 10},
        TacticUpdate(shadower, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    // Enemy has the ball but not the shadowee (same as first transition)
    // Robot should be trying to block possible pass to the shadowee
    fsm.process_event(ShadowEnemyFSM::Update(
        {enemy_threat, 0.5, 0.5},
        TacticUpdate(shadower, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ShadowEnemyFSM::BlockState>));
}

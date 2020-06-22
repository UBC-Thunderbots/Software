#include "software/ai/hl/stp/tactic/tactic_world_params_update_visitor.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(TacticUpdateVisitorTest, update_cherry_pick_tactic)
{
    World initial_world     = ::TestUtil::createBlankTestingWorld();
    Rectangle target_region = Rectangle(Point(1, 1), Point(0, 0));
    World update_world      = ::TestUtil::createBlankTestingWorld();
    update_world =
        ::TestUtil::setBallPosition(update_world, Point(1, 0), Timestamp::fromSeconds(0));
    CherryPickTactic tactic = CherryPickTactic(initial_world, target_region);
    EXPECT_EQ(tactic.getWorld(), initial_world);
    TacticWorldParamsUpdateVisitor visitor = TacticWorldParamsUpdateVisitor(update_world);
    tactic.accept(visitor);
    EXPECT_EQ(tactic.getWorld(), update_world);
}

TEST(TacticUpdateVisitorTest, update_chip_tactic)
{
    World initial_world = ::TestUtil::createBlankTestingWorld();
    World update_world  = ::TestUtil::createBlankTestingWorld();
    update_world =
        ::TestUtil::setBallPosition(update_world, Point(1, 0), Timestamp::fromSeconds(0));
    ChipTactic tactic = ChipTactic(initial_world.ball(), false);
    EXPECT_EQ(tactic.getBall(), initial_world.ball());
    TacticWorldParamsUpdateVisitor visitor = TacticWorldParamsUpdateVisitor(update_world);
    tactic.accept(visitor);
    EXPECT_EQ(tactic.getBall(), update_world.ball());
}

TEST(TacticUpdateVisitorTest, update_crease_defender_tactic)
{
    World initial_world = ::TestUtil::createBlankTestingWorld();
    World update_world  = ::TestUtil::createBlankTestingWorld();
    update_world =
        ::TestUtil::setBallPosition(update_world, Point(1, 0), Timestamp::fromSeconds(0));
    CreaseDefenderTactic tactic = CreaseDefenderTactic(
        initial_world.field(), initial_world.ball(), initial_world.friendlyTeam(),
        initial_world.enemyTeam(), CreaseDefenderTactic::LEFT);
    EXPECT_EQ(tactic.getField(), initial_world.field());
    EXPECT_EQ(tactic.getEnemyTeam(), initial_world.enemyTeam());
    EXPECT_EQ(tactic.getFriendlyTeam(), initial_world.friendlyTeam());
    EXPECT_EQ(tactic.getBall(), initial_world.ball());
    TacticWorldParamsUpdateVisitor visitor = TacticWorldParamsUpdateVisitor(update_world);
    tactic.accept(visitor);
    EXPECT_EQ(tactic.getField(), update_world.field());
    EXPECT_EQ(tactic.getEnemyTeam(), update_world.enemyTeam());
    EXPECT_EQ(tactic.getFriendlyTeam(), update_world.friendlyTeam());
    EXPECT_EQ(tactic.getBall(), update_world.ball());
}

TEST(TacticUpdateVisitorTest, update_defense_shadow_enemy)
{
    World initial_world = ::TestUtil::createBlankTestingWorld();
    World update_world  = ::TestUtil::createBlankTestingWorld();
    update_world =
        ::TestUtil::setBallPosition(update_world, Point(1, 0), Timestamp::fromSeconds(0));
    DefenseShadowEnemyTactic tactic = DefenseShadowEnemyTactic(
        initial_world.field(), initial_world.friendlyTeam(), initial_world.enemyTeam(),
        initial_world.ball(), true, 10.0);
    EXPECT_EQ(tactic.getField(), initial_world.field());
    EXPECT_EQ(tactic.getEnemyTeam(), initial_world.enemyTeam());
    EXPECT_EQ(tactic.getFriendlyTeam(), initial_world.friendlyTeam());
    EXPECT_EQ(tactic.getBall(), initial_world.ball());
    TacticWorldParamsUpdateVisitor visitor = TacticWorldParamsUpdateVisitor(update_world);
    tactic.accept(visitor);
    EXPECT_EQ(tactic.getField(), update_world.field());
    EXPECT_EQ(tactic.getEnemyTeam(), update_world.enemyTeam());
    EXPECT_EQ(tactic.getFriendlyTeam(), update_world.friendlyTeam());
    EXPECT_EQ(tactic.getBall(), update_world.ball());
}

TEST(TacticUpdateVisitorTest, update_goalie_tactic)
{
    World initial_world = ::TestUtil::createBlankTestingWorld();
    World update_world  = ::TestUtil::createBlankTestingWorld();
    update_world =
        ::TestUtil::setBallPosition(update_world, Point(1, 0), Timestamp::fromSeconds(0));
    GoalieTactic tactic =
        GoalieTactic(initial_world.ball(), initial_world.field(),
                     initial_world.friendlyTeam(), initial_world.enemyTeam());
    EXPECT_EQ(tactic.getField(), initial_world.field());
    EXPECT_EQ(tactic.getEnemyTeam(), initial_world.enemyTeam());
    EXPECT_EQ(tactic.getFriendlyTeam(), initial_world.friendlyTeam());
    EXPECT_EQ(tactic.getBall(), initial_world.ball());
    TacticWorldParamsUpdateVisitor visitor = TacticWorldParamsUpdateVisitor(update_world);
    tactic.accept(visitor);
    EXPECT_EQ(tactic.getField(), update_world.field());
    EXPECT_EQ(tactic.getEnemyTeam(), update_world.enemyTeam());
    EXPECT_EQ(tactic.getFriendlyTeam(), update_world.friendlyTeam());
    EXPECT_EQ(tactic.getBall(), update_world.ball());
}

TEST(TacticUpdateVisitorTest, update_passer_tactic)
{
    World initial_world = ::TestUtil::createBlankTestingWorld();
    World update_world  = ::TestUtil::createBlankTestingWorld();
    update_world =
        ::TestUtil::setBallPosition(update_world, Point(1, 0), Timestamp::fromSeconds(0));
    Pass pass({0, 0}, {0, -1}, 2.29, Timestamp::fromSeconds(5));
    PasserTactic tactic = PasserTactic(pass, initial_world.ball(), false);
    EXPECT_EQ(tactic.getBall(), initial_world.ball());
    TacticWorldParamsUpdateVisitor visitor = TacticWorldParamsUpdateVisitor(update_world);
    tactic.accept(visitor);
    EXPECT_EQ(tactic.getBall(), update_world.ball());
}

TEST(TacticUpdateVisitorTest, update_penalty_kick_tactic)
{
    World initial_world = ::TestUtil::createBlankTestingWorld();
    World update_world  = ::TestUtil::createBlankTestingWorld();
    update_world =
        ::TestUtil::setBallPosition(update_world, Point(1, 0), Timestamp::fromSeconds(0));
    Robot robot = Robot(3, Point(1, 1), Vector(-0.3, 0), Angle::fromRadians(2.2),
                        AngularVelocity::fromRadians(-0.6), Timestamp::fromSeconds(0));
    PenaltyKickTactic tactic = PenaltyKickTactic(
        initial_world.ball(), initial_world.field(), std::nullopt, true);
    EXPECT_EQ(tactic.getField(), initial_world.field());
    EXPECT_EQ(tactic.getBall(), initial_world.ball());
    TacticWorldParamsUpdateVisitor visitor = TacticWorldParamsUpdateVisitor(update_world);
    tactic.accept(visitor);
    EXPECT_EQ(tactic.getField(), update_world.field());
    EXPECT_EQ(tactic.getBall(), update_world.ball());
}

TEST(TacticUpdateVisitorTest, update_receiver_tactic)
{
    World initial_world = ::TestUtil::createBlankTestingWorld();
    World update_world  = ::TestUtil::createBlankTestingWorld();
    update_world =
        ::TestUtil::setBallPosition(update_world, Point(1, 0), Timestamp::fromSeconds(0));
    Pass pass({0, 0}, {0, -1}, 2.29, Timestamp::fromSeconds(5));
    ReceiverTactic tactic =
        ReceiverTactic(initial_world.field(), initial_world.friendlyTeam(),
                       initial_world.enemyTeam(), pass, initial_world.ball(), false);
    EXPECT_EQ(tactic.getField(), initial_world.field());
    EXPECT_EQ(tactic.getEnemyTeam(), initial_world.enemyTeam());
    EXPECT_EQ(tactic.getFriendlyTeam(), initial_world.friendlyTeam());
    EXPECT_EQ(tactic.getBall(), initial_world.ball());
    TacticWorldParamsUpdateVisitor visitor = TacticWorldParamsUpdateVisitor(update_world);
    tactic.accept(visitor);
    // ReceiverTactic doesn't update field
    EXPECT_EQ(tactic.getEnemyTeam(), update_world.enemyTeam());
    EXPECT_EQ(tactic.getFriendlyTeam(), update_world.friendlyTeam());
    EXPECT_EQ(tactic.getBall(), update_world.ball());
}

TEST(TacticUpdateVisitorTest, update_shadow_enemy_tactic)
{
    World initial_world = ::TestUtil::createBlankTestingWorld();
    World update_world  = ::TestUtil::createBlankTestingWorld();
    update_world =
        ::TestUtil::setBallPosition(update_world, Point(1, 0), Timestamp::fromSeconds(0));
    ShadowEnemyTactic tactic = ShadowEnemyTactic(
        initial_world.field(), initial_world.friendlyTeam(), initial_world.enemyTeam(),
        true, initial_world.ball(), 10.0, false, false);
    EXPECT_EQ(tactic.getField(), initial_world.field());
    EXPECT_EQ(tactic.getEnemyTeam(), initial_world.enemyTeam());
    EXPECT_EQ(tactic.getFriendlyTeam(), initial_world.friendlyTeam());
    EXPECT_EQ(tactic.getBall(), initial_world.ball());
    TacticWorldParamsUpdateVisitor visitor = TacticWorldParamsUpdateVisitor(update_world);
    tactic.accept(visitor);
    EXPECT_EQ(tactic.getField(), update_world.field());
    EXPECT_EQ(tactic.getEnemyTeam(), update_world.enemyTeam());
    EXPECT_EQ(tactic.getFriendlyTeam(), update_world.friendlyTeam());
    EXPECT_EQ(tactic.getBall(), update_world.ball());
}

TEST(TacticUpdateVisitorTest, update_shadow_freekicker_tactic)
{
    World initial_world = ::TestUtil::createBlankTestingWorld();
    World update_world  = ::TestUtil::createBlankTestingWorld();
    update_world =
        ::TestUtil::setBallPosition(update_world, Point(1, 0), Timestamp::fromSeconds(0));
    ShadowFreekickerTactic tactic =
        ShadowFreekickerTactic(ShadowFreekickerTactic::LEFT, initial_world.enemyTeam(),
                               initial_world.ball(), initial_world.field(), false);
    EXPECT_EQ(tactic.getField(), initial_world.field());
    EXPECT_EQ(tactic.getEnemyTeam(), initial_world.enemyTeam());
    EXPECT_EQ(tactic.getBall(), initial_world.ball());
    TacticWorldParamsUpdateVisitor visitor = TacticWorldParamsUpdateVisitor(update_world);
    tactic.accept(visitor);
    // ShadowFreeKickerTactic doesn't update field
    EXPECT_EQ(tactic.getEnemyTeam(), update_world.enemyTeam());
    EXPECT_EQ(tactic.getBall(), update_world.ball());
}

TEST(TacticUpdateVisitorTest, update_shoot_goal_tactic)
{
    World initial_world = ::TestUtil::createBlankTestingWorld();
    World update_world  = ::TestUtil::createBlankTestingWorld();
    update_world =
        ::TestUtil::setBallPosition(update_world, Point(1, 0), Timestamp::fromSeconds(0));
    ShootGoalTactic tactic = ShootGoalTactic(
        initial_world.field(), initial_world.friendlyTeam(), initial_world.enemyTeam(),
        initial_world.ball(), Angle::zero(), std::nullopt, false);
    EXPECT_EQ(tactic.getField(), initial_world.field());
    EXPECT_EQ(tactic.getEnemyTeam(), initial_world.enemyTeam());
    EXPECT_EQ(tactic.getFriendlyTeam(), initial_world.friendlyTeam());
    EXPECT_EQ(tactic.getBall(), initial_world.ball());
    TacticWorldParamsUpdateVisitor visitor = TacticWorldParamsUpdateVisitor(update_world);
    tactic.accept(visitor);
    EXPECT_EQ(tactic.getField(), update_world.field());
    EXPECT_EQ(tactic.getEnemyTeam(), update_world.enemyTeam());
    EXPECT_EQ(tactic.getFriendlyTeam(), update_world.friendlyTeam());
    EXPECT_EQ(tactic.getBall(), update_world.ball());
}

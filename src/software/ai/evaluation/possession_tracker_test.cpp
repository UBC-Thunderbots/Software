#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/possession.h"
#include "software/test_util/test_util.h"


TEST(PossessionTrackerTest, get_possession_with_ball_near_team)
{
    auto world = TestUtil::createBlankTestingWorld();

    auto friendly_team = TestUtil::setRobotPositionsHelper(
        world.friendlyTeam(), {Point(0.5, 0)}, Timestamp::fromSeconds(0));
    auto enemy_team = TestUtil::setRobotPositionsHelper(
        world.enemyTeam(), {Point(1.5, 0)}, Timestamp::fromSeconds(0));
    world.updateFriendlyTeamState(friendly_team);
    world.updateEnemyTeamState(enemy_team);

    TbotsProto::PossessionTrackerConfig config;
    PossessionTracker possession_tracker(config);
    TeamPossession possession;

    // Ball equally far away from both friendly and enemy bots
    // (i.e neither team has presence over the ball).
    // Friendly team should have possession since we should seek to gain
    // possession of the ball.
    world.updateBall(Ball({1, 0}, {0, 0}, Timestamp::fromSeconds(0)));
    possession = possession_tracker.getTeamWithPossession(
        world.friendlyTeam(), world.enemyTeam(), world.ball(), world.field());
    EXPECT_EQ(possession, TeamPossession::FRIENDLY_TEAM);

    // Move ball near enemy bot for a period of time.
    // Enemy team should have clear possession.
    world.updateBall(Ball({1.45, 0}, {0, 0}, Timestamp::fromSeconds(0)));
    possession = possession_tracker.getTeamWithPossession(
        world.friendlyTeam(), world.enemyTeam(), world.ball(), world.field());
    world.updateBall(Ball({1.45, 0}, {0, 0}, Timestamp::fromSeconds(0.5)));
    possession = possession_tracker.getTeamWithPossession(
        world.friendlyTeam(), world.enemyTeam(), world.ball(), world.field());
    EXPECT_EQ(possession, TeamPossession::ENEMY_TEAM);

    // Move ball near friendly bot for a period of time.
    // Friendly team should have clear possession.
    world.updateBall(Ball({0.55, 0}, {0, 0}, Timestamp::fromSeconds(0.5)));
    possession = possession_tracker.getTeamWithPossession(
        world.friendlyTeam(), world.enemyTeam(), world.ball(), world.field());
    world.updateBall(Ball({0.55, 0}, {0, 0}, Timestamp::fromSeconds(1)));
    possession = possession_tracker.getTeamWithPossession(
        world.friendlyTeam(), world.enemyTeam(), world.ball(), world.field());
    EXPECT_EQ(possession, TeamPossession::FRIENDLY_TEAM);
}

TEST(PossessionTrackerTest, get_possession_with_ball_near_both_teams)
{
    auto world = TestUtil::createBlankTestingWorld();

    // Position all robots in the friendly half.
    auto friendly_team = TestUtil::setRobotPositionsHelper(
        world.friendlyTeam(), {Point(-0.45, 0)}, Timestamp::fromSeconds(0));
    auto enemy_team = TestUtil::setRobotPositionsHelper(
        world.enemyTeam(), {Point(-0.55, 0)}, Timestamp::fromSeconds(0));
    world.updateFriendlyTeamState(friendly_team);
    world.updateEnemyTeamState(enemy_team);

    TbotsProto::PossessionTrackerConfig config;
    PossessionTracker possession_tracker(config);
    TeamPossession possession;

    // Ball equally near both friendly and enemy bots for a period of time
    // (i.e both teams have presence over the ball).
    // Enemy team should have possession since the ball is being fought
    // over in the friendly half.
    world.updateBall(Ball({-0.5, 0}, {0, 0}, Timestamp::fromSeconds(0)));
    possession = possession_tracker.getTeamWithPossession(
        world.friendlyTeam(), world.enemyTeam(), world.ball(), world.field());
    world.updateBall(Ball({-0.5, 0}, {0, 0}, Timestamp::fromSeconds(1)));
    possession = possession_tracker.getTeamWithPossession(
        world.friendlyTeam(), world.enemyTeam(), world.ball(), world.field());
    EXPECT_EQ(possession, TeamPossession::ENEMY_TEAM);

    // Position all bots in the enemy half.
    friendly_team = TestUtil::setRobotPositionsHelper(
        world.friendlyTeam(), {Point(0.45, 0)}, Timestamp::fromSeconds(1));
    enemy_team = TestUtil::setRobotPositionsHelper(world.enemyTeam(), {Point(0.55, 0)},
                                                   Timestamp::fromSeconds(1));
    world.updateFriendlyTeamState(friendly_team);
    world.updateEnemyTeamState(enemy_team);

    // Ball equally near both friendly and enemy bots for a period of time.
    // Friendly team should have possession since the ball is in the enemy half and
    // there are no enemies in the friendly half.
    world.updateBall(Ball({0.5, 0}, {0, 0}, Timestamp::fromSeconds(1)));
    possession = possession_tracker.getTeamWithPossession(
        world.friendlyTeam(), world.enemyTeam(), world.ball(), world.field());
    world.updateBall(Ball({0.5, 0}, {0, 0}, Timestamp::fromSeconds(2)));
    possession = possession_tracker.getTeamWithPossession(
        world.friendlyTeam(), world.enemyTeam(), world.ball(), world.field());
    EXPECT_EQ(possession, TeamPossession::FRIENDLY_TEAM);
}

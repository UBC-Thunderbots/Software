#include "play_monitor.h"


PlayMonitor::PlayMonitor() : startingWorld_() {}

void PlayMonitor::startMonitoring(const World& initialWorld)
{
    startingWorld_ = initialWorld;
}

double PlayMonitor::endMonitoring(const World& finalWorld)
{
    return calculateCurrentPlayScore(finalWorld);
}

void PlayMonitor::updateWorld(const World& newWorld)
{
    startingWorld_ = newWorld;
}

double PlayMonitor::calculateCurrentPlayScore(const World& endingWorld) const
{
    const auto attackingTeam = endingWorld.getTeamWithPossession();
    // Friendlies scored the ball
    if (contains(endingWorld.field().enemyGoal(), endingWorld.ball().position()))
    {
        return 1.0;
    }
    if (attackingTeam == TeamPossession::FRIENDLY_TEAM)
    {
        // How far the ball has travelled along the x axis between updatedWorld and the
        // old world, this indicates that generally the ball has move closer to the enemy
        // goal.
        const auto starting_ball_position = startingWorld_.ball().position().x();
        const auto final_ball_position    = endingWorld.ball().position().x();
        const auto ball_distance_travelled_along_field =
            final_ball_position - starting_ball_position;
        return ball_distance_travelled_along_field / startingWorld_.field().xLength();
    }

    if (attackingTeam == TeamPossession::ENEMY_TEAM &&
        contains(startingWorld_.field().enemyHalf(), startingWorld_.ball().position()))
    {
        return 0.0;
    }

    return 0.0;
}

#include "software/ai/hl/stp/play/monitor/play_monitor.h"




PlayMonitor::PlayMonitor(const PlayIntent& initialIntent)
    : intent(initialIntent), world()
{
}

void PlayMonitor::startMonitoring(const World &initialWorld)
{
    world  = initialWorld;
}

double PlayMonitor::endMonitoring(const World &finalWorld)
{
    return calculateCurrentPlayScore(finalWorld);
}

void PlayMonitor::updateWorld(const World& newWorld)
{
    world = newWorld;
}

void PlayMonitor::updatePlayIntent(PlayIntent newIntent)
{
    intent = newIntent;
}

double PlayMonitor::calculateCurrentPlayScore(const World & finalWorld)
{
    // Friendlies scored the ball
    if (contains(finalWorld.field().enemyGoal(), finalWorld.ball().position()))
    {
        return 2.0;
    }


    const auto attackingTeam = finalWorld.getTeamWithPossession();
    if (attackingTeam == TeamPossession::FRIENDLY_TEAM)
    {
        // How far the ball has travelled between updatedWorld and the old world
        const auto ball_distance_travelled = distance(finalWorld.ball().position(), world.ball().position());
        // How far the ball has travelled along the x axis between updatedWorld and the old world,
        // this indicates that generally the ball has move closer to the enemy goal.
        // this could probably later be revised later by also considering the y value,
        // when the ball is in the enemy third - at that point the play should try to
        // funnel the ball closer to the goal
        const auto ball_distance_travelled_along_field = finalWorld.ball().position().x() - world.ball().position().x();
        if (ball_distance_travelled > (4 * BALL_MAX_RADIUS_METERS))
        {
            if (intent.getIntentType() == PASS)
            {
                return 1.0;
            }
            //
            return ball_distance_travelled_along_field / finalWorld.field().xLength();
        }

        // we still have possesion, but nothing significant has changed.
        return  0.0;
    }

    if (attackingTeam == TeamPossession::ENEMY_TEAM &&
        contains(world.field().enemyHalf(), world.ball().position()))
    {
        return -1.0;
    }
    if (attackingTeam == TeamPossession::ENEMY_TEAM &&
    contains(world.field().enemyHalf(), world.ball().position()))
    {
        return -1.0;
    }
    LOG(WARNING) << "DribblerMode is invalid" << std::endl;
    return  0.0;
}
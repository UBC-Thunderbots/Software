#include "software/ai/evaluation/possession.h"

#include "shared/constants.h"
#include "software/ai/evaluation/intercept.h"
#include "software/geom/algorithms/distance.h"
#include "software/world/ball.h"
#include "software/world/field.h"
#include "software/world/team.h"

PossessionTracker::PossessionTracker() {}

TeamPossession PossessionTracker::getTeamWithPossession(const Team &friendly_team,
                                                        const Team &enemy_team,
                                                        const Ball &ball,
                                                        const Field &field)
{
    // Based on CMDragons TDP 2015

    updateTimes(friendly_team, enemy_team, ball, field);

    const Duration TIME_NEAR_THRESHOLD = Duration::fromSeconds(0.1);
    const Duration TIME_FAR_THRESHOLD = Duration::fromSeconds(1.5);
    
    if ((time_near_friendly > TIME_NEAR_THRESHOLD) && (time_near_enemy < TIME_NEAR_THRESHOLD))
    {
        possession = TeamPossession::FRIENDLY_TEAM;
    }
    else if ((time_near_friendly < TIME_NEAR_THRESHOLD) && (time_near_enemy > TIME_NEAR_THRESHOLD))
    {
        possession = TeamPossession::ENEMY_TEAM;
    }
    else if ((time_near_friendly > TIME_NEAR_THRESHOLD) && (time_near_enemy > TIME_NEAR_THRESHOLD))
    {
        // Both teams are considered to have presence over the ball.
        // Determine possession based on which side of the field the ball is on.
        if (field.pointInFriendlyHalf(ball.position()))
        {
            possession = TeamPossession::ENEMY_TEAM;
        }
        else
        {
            possession = TeamPossession::FRIENDLY_TEAM;
        }
    }
    else if ((time_far_friendly > TIME_FAR_THRESHOLD) && (time_far_enemy > TIME_FAR_THRESHOLD))
    {
        // No team has presence over the ball.
        // Consider our team as having possession since we should seek to gain
        // possession of the ball. 
        possession = TeamPossession::FRIENDLY_TEAM;
    }

    return possession;
}

void PossessionTracker::updateTimes(const Team &friendly_team,
                                    const Team &enemy_team,
                                    const Ball &ball,
                                    const Field &field)
{
    Duration delta_time = ball.timestamp() - last_timestamp;
    last_timestamp = ball.timestamp();

    auto nearest_friendly = getRobotWithEffectiveBallPossession(friendly_team, ball, field);
    auto nearest_enemy = getRobotWithEffectiveBallPossession(enemy_team, ball, field);
    if (!nearest_friendly || !nearest_enemy)
    {
        return;
    }

    double distance_friendly = distance(nearest_friendly->position(), ball.position());
    double distance_enemy = distance(nearest_enemy->position(), ball.position());

    if (distance_friendly < DISTANCE_NEAR_TOLERANCE)
    {
        time_near_friendly = time_near_friendly + delta_time;
    }
    else
    {
        time_near_friendly = Duration::fromSeconds(0);
    }

    if (distance_enemy < DISTANCE_NEAR_TOLERANCE)
    {
        time_near_enemy = time_near_enemy + delta_time;
    }
    else
    {
        time_near_enemy = Duration::fromSeconds(0);
    }

    if (distance_friendly > DISTANCE_FAR_TOLERANCE)
    {
        time_far_friendly = time_far_friendly + delta_time;
    }
    else
    {
        time_far_friendly = Duration::fromSeconds(0);
    }

    if (distance_enemy > DISTANCE_FAR_TOLERANCE)
    {
        time_far_enemy = time_far_enemy + delta_time;
    }
    else
    {
        time_far_enemy = Duration::fromSeconds(0);
    }
}

std::optional<Robot> getRobotWithEffectiveBallPossession(const Team &team,
                                                         const Ball &ball,
                                                         const Field &field)
{
    if (team.numRobots() == 0)
    {
        return std::nullopt;
    }

    auto best_intercept =
        findBestInterceptForBall(ball, field, team.getAllRobots().at(0));
    auto baller_robot = team.getAllRobots().at(0);

    // Find the robot that can intercept the ball the quickest
    for (const auto &robot : team.getAllRobots())
    {
        auto intercept = findBestInterceptForBall(ball, field, robot);
        if (!best_intercept || (intercept && intercept->second < best_intercept->second))
        {
            best_intercept = intercept;
            baller_robot   = robot;
        }
    }

    // Return the robot that can intercept the ball the fastest within the field. If
    // no robot is able to intercept the ball within the field, return the closest
    // robot to the ball
    if (best_intercept)
    {
        return baller_robot;
    }
    else
    {
        return team.getNearestRobot(ball.position());
    }
}

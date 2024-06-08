#include "software/sensor_fusion/possession/possession_tracker.h"

#include "shared/constants.h"
#include "software/ai/evaluation/possession.h"
#include "software/geom/algorithms/distance.h"

PossessionTracker::PossessionTracker(const TbotsProto::PossessionTrackerConfig &config)
    : distance_near_tolerance_meters(config.distance_near_tolerance_meters()),
      distance_far_tolerance_meters(config.distance_far_tolerance_meters()),
      time_near_threshold(Duration::fromSeconds(config.time_near_threshold_s())),
      time_far_threshold(Duration::fromSeconds(config.time_far_threshold_s())),
      last_timestamp(Timestamp::fromSeconds(0)),
      time_near_friendly(Duration::fromSeconds(0)),
      time_near_enemy(Duration::fromSeconds(0)),
      time_far_friendly(Duration::fromSeconds(0)),
      time_far_enemy(Duration::fromSeconds(0)),
      possession(TeamPossession::LOOSE)
{
}

TeamPossession PossessionTracker::getTeamWithPossession(const Team &friendly_team,
                                                        const Team &enemy_team,
                                                        const Ball &ball,
                                                        const Field &field)
{
    // Based on CMDragons TDP 2015
    // http://www.cs.cmu.edu/~jmendoza/papers/cmdragons_robocup15.pdf

    updateTimes(friendly_team, enemy_team, ball, field);

    if ((time_near_friendly > time_near_threshold) &&
        (time_near_enemy < time_near_threshold))
    {
        possession = TeamPossession::FRIENDLY;
    }
    else if ((time_near_friendly < time_near_threshold) &&
             (time_near_enemy > time_near_threshold))
    {
        possession = TeamPossession::ENEMY;
    }
    else if ((time_near_friendly > time_near_threshold) &&
             (time_near_enemy > time_near_threshold))
    {
        // Both teams are considered to have presence over the ball.
        possession = TeamPossession::IN_CONTEST;
    }
    else if ((time_far_friendly > time_far_threshold) &&
             (time_far_enemy > time_far_threshold))
    {
        // No team has presence over the ball.
        possession = TeamPossession::LOOSE;
    }

    return possession;
}

void PossessionTracker::updateTimes(const Team &friendly_team, const Team &enemy_team,
                                    const Ball &ball, const Field &field)
{
    Duration delta_time = ball.timestamp() - last_timestamp;
    last_timestamp      = ball.timestamp();

    auto nearest_friendly =
        getRobotWithEffectiveBallPossession(friendly_team, ball, field);
    auto nearest_enemy = getRobotWithEffectiveBallPossession(enemy_team, ball, field);
    if (!nearest_friendly || !nearest_enemy)
    {
        return;
    }

    double distance_friendly = distance(nearest_friendly->position(), ball.position());
    double distance_enemy    = distance(nearest_enemy->position(), ball.position());

    if (distance_friendly < distance_near_tolerance_meters)
    {
        time_near_friendly = time_near_friendly + delta_time;
    }
    else
    {
        time_near_friendly = Duration::fromSeconds(0);
    }

    if (distance_enemy < distance_near_tolerance_meters)
    {
        time_near_enemy = time_near_enemy + delta_time;
    }
    else
    {
        time_near_enemy = Duration::fromSeconds(0);
    }

    if (distance_friendly > distance_far_tolerance_meters)
    {
        time_far_friendly = time_far_friendly + delta_time;
    }
    else
    {
        time_far_friendly = Duration::fromSeconds(0);
    }

    if (distance_enemy > distance_far_tolerance_meters)
    {
        time_far_enemy = time_far_enemy + delta_time;
    }
    else
    {
        time_far_enemy = Duration::fromSeconds(0);
    }
}

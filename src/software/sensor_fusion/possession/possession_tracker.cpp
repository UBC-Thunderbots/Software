#include "software/sensor_fusion/possession/possession_tracker.h"

#include "shared/constants.h"
#include "software/ai/evaluation/possession.h"
#include "software/geom/algorithms/distance.h"

PossessionTracker::PossessionTracker(const TbotsProto::PossessionTrackerConfig &config)
    : distance_near_tolerance_meters(config.distance_near_tolerance_meters()),
      distance_far_tolerance_meters(config.distance_far_tolerance_meters()),
      ball_moved_threshold_meters(config.ball_moved_threshold_meters()),
      time_near_threshold(Duration::fromSeconds(config.time_near_threshold_s())),
      time_far_threshold(Duration::fromSeconds(config.time_far_threshold_s())),
      time_stagnant_threshold(Duration::fromSeconds(config.time_stagnant_threshold_s())),
      time_ball_stagnant_threshold(Duration::fromSeconds(config.time_ball_stagnant_threshold_s())),
      last_timestamp(Timestamp::fromSeconds(0)),
      time_near_friendly(Duration::fromSeconds(0)),
      time_near_enemy(Duration::fromSeconds(0)),
      time_far_friendly(Duration::fromSeconds(0)),
      time_far_enemy(Duration::fromSeconds(0)),
      time_since_ball_moved(Duration::fromSeconds(0)),
      last_ball_position(Point()),
      possession(TeamPossession::FRIENDLY_TEAM)
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
        possession = TeamPossession::FRIENDLY_TEAM;
    }
    else if ((time_near_friendly < time_near_threshold) &&
             (time_near_enemy > time_near_threshold))
    {
        if (time_near_enemy < time_stagnant_threshold &&
            time_since_ball_moved < time_ball_stagnant_threshold)
        {
            possession = TeamPossession::ENEMY_TEAM;
        }
        else
        {
            possession = TeamPossession::STAGNANT_ENEMY_TEAM;
        }
    }
    else if ((time_near_friendly > time_near_threshold) &&
             (time_near_enemy > time_near_threshold))
    {
        // Both teams are considered to have presence over the ball.
        // If the ball is on our side of the field, or there are enemy robots
        // on our side of the field, consider enemy team as having possession.

        auto enemy_team_robots = enemy_team.getAllRobotsExceptGoalie();
        auto num_enemies_in_friendly_half =
            std::count_if(enemy_team_robots.begin(), enemy_team_robots.end(),
                          [&field](const auto &enemy) {
                              return field.pointInFriendlyHalf(enemy.position());
                          });

        if (field.pointInFriendlyHalf(ball.position()) ||
            num_enemies_in_friendly_half > 0)
        {
            // Possession is considered stagnant since the ball is being
            // actively over fought over (both teams are near ball), but
            // enemy possession remains unchanged
            possession = TeamPossession::STAGNANT_ENEMY_TEAM;
        }
        else
        {
            possession = TeamPossession::FRIENDLY_TEAM;
        }
    }
    else if ((time_far_friendly > time_far_threshold) &&
             (time_far_enemy > time_far_threshold))
    {
        // No team has presence over the ball.
        // Consider our team as having possession since we should seek to gain
        // possession of the ball.
        possession = TeamPossession::FRIENDLY_TEAM;
    }

    return possession;
}

void PossessionTracker::updateTimes(const Team &friendly_team, const Team &enemy_team,
                                    const Ball &ball, const Field &field)
{
    Duration delta_time = ball.timestamp() - last_timestamp;
    last_timestamp      = ball.timestamp();

    // Check whether the ball moved and update the amount of time 
    // since the ball last moved
    if (distance(ball.position(), last_ball_position) > ball_moved_threshold_meters)
    {
        time_since_ball_moved = Duration::fromSeconds(0);
    }
    else
    {
        time_since_ball_moved = time_since_ball_moved + delta_time;
    }
    last_ball_position = ball.position();

    auto nearest_friendly =
        getRobotWithEffectiveBallPossession(friendly_team, ball, field);
    auto nearest_enemy = getRobotWithEffectiveBallPossession(enemy_team, ball, field);

    if (!nearest_enemy)
    {
        // No enemy threats, so zero time spent near and away from enemy.
        // This will ensure that the friendly team has possession.
        time_near_enemy = Duration::fromSeconds(0);
        time_far_enemy = Duration::fromSeconds(0);
        return;
    }

    if (!nearest_friendly)
    {
        // No friendly robots, so zero time spent near and away from friendly team.
        // This will ensure that the enemy team has possession.
        time_near_friendly = Duration::fromSeconds(0);
        time_far_friendly = Duration::fromSeconds(0);
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

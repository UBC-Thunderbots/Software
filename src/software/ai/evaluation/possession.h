#pragma once

#include "software/time/duration.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

enum TeamPossession
{
    FRIENDLY_TEAM,
    ENEMY_TEAM
};

class PossessionTracker
{
   public:
    explicit PossessionTracker();

    static constexpr double DISTANCE_NEAR_TOLERANCE = 0.1;
    static constexpr double DISTANCE_FAR_TOLERANCE = 0.5;
    
    TeamPossession getTeamWithPossession(const Team &friendly_team,
                                         const Team &enemy_team,
                                         const Ball &ball,
                                         const Field &field);

   private:
    Timestamp last_timestamp;
    Duration time_near_friendly;
    Duration time_near_enemy;
    Duration time_far_friendly;
    Duration time_far_enemy;
    TeamPossession possession;

    void updateTimes(const Team &friendly_team,
                     const Team &enemy_team,
                     const Ball &ball,
                     const Field &field);
};

/**
 * Returns the robot that either has the ball, or is the closest to having it (and
 * therefore has the most "presence" over the ball)
 *
 * @param team The team containing the robots to check for possession
 * @param ball the Ball
 * @param field The Field being played on
 * @return the robot that either has the ball, or is the closest to having it. If the
 * team has no robots, std::nullopt is returned
 */
std::optional<Robot> getRobotWithEffectiveBallPossession(const Team &team,
                                                         const Ball &ball,
                                                         const Field &field);

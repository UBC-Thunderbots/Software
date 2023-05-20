#pragma once

#include "software/time/duration.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

// Indicates which team has possession of the ball
enum TeamPossession
{
    FRIENDLY_TEAM,
    ENEMY_TEAM
};

/**
 * Class to keep track of the state of the game over time and determine which
 * team has possession of the ball. The logic for determining possession state
 * is based on that used by TIGERs Mannheim, who borrow from CMDragons' 2015 TDP.
 */
class PossessionTracker
{
   public:
    explicit PossessionTracker();

    // Max distance in meters between robot and ball for robot
    // to be considered near the ball
    static constexpr double DISTANCE_NEAR_TOLERANCE_METERS = 0.1;

    // Min distance in meters between robot and ball for robot
    // to be considered far away from the ball
    static constexpr double DISTANCE_FAR_TOLERANCE_METERS = 0.5;

    // Min time in seconds that robot must stay close to ball for
    // it to be considered near
    static constexpr double TIME_NEAR_THRESHOLD_S = 0.1;

    // Min time in seconds that robot must stay away from ball for
    // it to be considered far away
    static constexpr double TIME_FAR_THRESHOLD_S = 0.5;

    /**
     * Returns a TeamPossession value indicating which team has possession of the ball.
     *
     * @param friendly_team the friendly team
     * @param enemy_team the enemy team
     * @param ball the ball
     * @param field the field being played on
     * @return a TeamPossession value indicating which team has possession of the ball
     */
    TeamPossession getTeamWithPossession(const Team &friendly_team,
                                         const Team &enemy_team, const Ball &ball,
                                         const Field &field);

   private:
    Timestamp last_timestamp;
    Duration time_near_friendly;
    Duration time_near_enemy;
    Duration time_far_friendly;
    Duration time_far_enemy;
    TeamPossession possession;

    void updateTimes(const Team &friendly_team, const Team &enemy_team, const Ball &ball,
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

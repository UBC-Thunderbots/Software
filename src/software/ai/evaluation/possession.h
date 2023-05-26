#pragma once

#include "proto/parameters.pb.h"
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
 *
 * CMDragons 2015 TDP
 * http://www.cs.cmu.edu/~jmendoza/papers/cmdragons_robocup15.pdf
 */
class PossessionTracker
{
   public:
    explicit PossessionTracker(const TbotsProto::PossessionTrackerConfig &config);

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

    // Max distance in meters between robot and ball for robot
    // to be considered near the ball
    double distance_near_tolerance_meters;

    // Min distance in meters between robot and ball for robot
    // to be considered far away from the ball
    double distance_far_tolerance_meters;

    // Min time in seconds that robot must stay close to ball for
    // it to be considered near
    Duration time_near_threshold;

    // Min time in seconds that robot must stay away from ball for
    // it to be considered far away
    Duration time_far_threshold;

    // The timestamp of last game snapshot the PossessionTracker was updated with
    Timestamp last_timestamp;

    // The amount of time that the friendly team has spent near the ball
    Duration time_near_friendly;

    // The amount of time that the enemy team has spent near the ball
    Duration time_near_enemy;

    // The amount of time that the friendly team has spent far away from the ball
    Duration time_far_friendly;

    // The amount of time that the enemy team has spent far away from the ball
    Duration time_far_enemy;

    // The team currently with possession of the ball
    TeamPossession possession;
    
    /**
     * Updates the amounts of time that each team has spent near or away from the ball
     *
     * @param friendly_team the friendly team
     * @param enemy_team the enemy team
     * @param ball the ball
     * @param field the field being played on
     */
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

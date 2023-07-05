#pragma once

#include "proto/parameters.pb.h"
#include "software/time/duration.h"
#include "software/time/timestamp.h"
#include "software/world/ball.h"
#include "software/world/field.h"
#include "software/world/team.h"

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

    // Min distance in meters that ball must move between two timestamps
    // for the ball to be considered moved
    double ball_moved_threshold_meters;

    // Min time that robot must stay close to ball for it to be considered near
    Duration time_near_threshold;

    // Min time that robot must stay away from ball for it to be considered far away
    Duration time_far_threshold;

    // Min time that team must hold possession for in order to be considered stagnant
    Duration time_stagnant_threshold;

    // Min time that the ball must stay still in order for possession to be considered stagnant
    Duration time_ball_stagnant_threshold;

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

    // The amount of time since the ball last moved
    Duration time_since_ball_moved;

    // The position of the ball at the last timestamp
    Point last_ball_position;

    // The team currently with possession of the ball
    TeamPossession possession;

    /**
     * Updates the amounts of time that each team has spent near or away from the ball
     * and the amount of time since the ball last moved
     *
     * @param friendly_team the friendly team
     * @param enemy_team the enemy team
     * @param ball the ball
     * @param field the field being played on
     */
    void updateTimes(const Team &friendly_team, const Team &enemy_team, const Ball &ball,
                     const Field &field);
};

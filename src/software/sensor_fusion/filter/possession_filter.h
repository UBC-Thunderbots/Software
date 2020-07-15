#pragma once

#include <vector>

#include "shared/constants.h"
#include "software/world/ball.h"
#include "software/world/robot_state.h"
#include "software/world/team.h"
#include "software/world/timestamped_possession_state.h"

/**
 * PossessionFilter is a filter that determines which robots have possession of the ball
 */
class PossessionFilter
{
   public:
    /**
     * Creates a PossessionFilter
     *
     * @param possession_threshold_meters The distance between ball position and robot
     * position to determine if the robot has possession of the ball. The default value is
     * experimentally determined to be a reasonable value
     */
    PossessionFilter(double possession_threshold_meters = ROBOT_MAX_RADIUS_METERS + 0.2);

    /**
     * Compiles the list of robots that have possession of the ball
     *
     * @param friendly_robots_with_breakbeam_triggered The friendly robots that have their
     * breakbeam triggered
     * @param friendly_team The friendly_team of robots
     * @param enemy_team The enemy_team of robots
     * @param ball The ball
     *
     * @return the list of robots that possession of the ball
     */
    std::vector<RobotIdWithTeamSide> getRobotsWithPossession(
        std::vector<RobotId> friendly_robots_with_breakbeam_triggered, Team friendly_team,
        Team enemy_team, Ball ball) const;

   private:
    /**
     * Decides if the ball is near the dribbler of the robot
     *
     * @param ball_position The position of the ball
     * @param robot_position The position of the robot
     * @param robot_orientation The orientation the robot
     *
     * @return whether the ball is near the dribbler of the robot
     */
    bool ballNearDribbler(Point ball_position, Point robot_position,
                          Angle robot_orientation) const;

    double possession_threshold_meters;
};

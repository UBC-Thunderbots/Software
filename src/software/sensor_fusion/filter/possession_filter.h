#pragma once

#include <vector>

#include "shared/constants.h"
#include "software/sensor_fusion/filter/vision_detection.h"
#include "software/world/ball.h"
#include "software/world/robot_state.h"
#include "software/world/team.h"
#include "software/world/team_types.h"

/**
 * Finds the robot that has possession of the ball. To have possession, the robot needs to
 * be close to the ball and the closest of all robots on the team
 *
 * @param ball The ball
 * @param team The team
 * @param robots_with_breakbeam_triggered The robots that have their breakbeam triggered
 * @param possession_threshold The threshold distance between the ball and robot at which
 * the robot is considered to have possession
 *
 * @return The robot with possession of the ball
 */
std::optional<RobotId> getRobotWithPossession(
    const Ball &ball, const Team &team,
    const std::vector<RobotId> &robots_with_breakbeam_triggered = std::vector<RobotId>(),
    double possession_distance_threshold = ROBOT_MAX_RADIUS_METERS + 0.2);

/**
 * Gets the distance between the robot and the ball if the robot has possession of the
 * ball
 *
 * @param ball_position The position of the ball
 * @param robot_position The position of the robot
 * @param robot_orientation The orientation the robot
 * @param possession_threshold The threshold distance between the ball and robot at which
 * the robot is considered to have possession
 *
 * return the distance between the robot and the ball if it has possession
 */
std::optional<double> getPossessionDistance(
    Point ball_position, Point robot_position, Angle robot_orientation,
    double possession_distance_threshold = ROBOT_MAX_RADIUS_METERS + 0.2);

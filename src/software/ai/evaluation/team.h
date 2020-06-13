#pragma once

#include "software/world/robot.h"
#include "software/world/team.h"

/**
 * Given a team, finds the robot on that team that is closest to a reference point.
 *
 * @param team the team of robots
 * @param ref_point The point where the distance to each robot will be measured.
 * @return Robot that is closest to the reference point.
 */
std::optional<Robot> nearestRobot(const Team &team, const Point &ref_point);

/**
 * Given a list of robots, finds the robot on that team that is closest to a
 * reference point.
 *
 * @param robots the list of robots
 * @param ref_point The point where the distance to each robot will be measured.
 * @return Robot that is closest to the reference point.
 */
std::optional<Robot> nearestRobot(const std::vector<Robot> &robots,
                                  const Point &ref_point);

#pragma once

#include "software/world/world.h"

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

/**
 * Returns the team that either has possession of the ball, or is the closest to
 * having it (and therefore has the most "presence" over the ball)
 *
 * @param friendly_team the friendly team
 * @param enemy_team the enemy team
 * @param ball the ball
 * @param field the field being played on
 * @return the team that either has the ball, or is the closest to having it. If
 * both teams have no robots, std::nullopt is returned
 */
std::optional<Team> getTeamWithEffectiveBallPossession(const Team &friendly_team,
                                                       const Team &enemy_team,
                                                       const Ball &ball,
                                                       const Field &field);

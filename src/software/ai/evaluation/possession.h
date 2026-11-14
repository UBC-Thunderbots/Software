#pragma once

#include "proto/parameters.pb.h"
#include "software/time/duration.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

/**
 * Returns the robot that either has the ball, or is the closest to having it (and
 * therefore has the most "presence" over the ball)
 *
 * @param team The team containing the robots to check for possession
 * @param ball the Ball
 * @param field The Field being played on
 * @param previous_baller The robot returned on the previous update, if any. It stays the
 * result unless another robot can intercept the ball more than switch_margin sooner
 * @param switch_margin How much sooner another robot must reach the ball to replace
 * previous_baller
 * @return the robot that either has the ball, or is the closest to having it. If the
 * team has no robots, std::nullopt is returned
 */
std::optional<Robot> getRobotWithEffectiveBallPossession(
    const Team& team, const Ball& ball, const Field& field,
    std::optional<RobotId> previous_baller = std::nullopt,
    const Duration& switch_margin = Duration::fromSeconds(0));

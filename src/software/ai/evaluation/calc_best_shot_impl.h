#pragma once

#include "shared/constants.h"
#include "software/ai/evaluation/shot.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"
#include "software/util/make_enum/make_enum.h"
#include "software/world/field.h"
#include "software/world/robot.h"
#include "software/world/team.h"
#include "software/world/world.h"

/**
 * Calculates the percentage of the net that a given shot is aiming for
 *
 * @param field The field the shot is being performed on
 * @param shot_origin The point the shot is being taken from
 * @param shot The shot being performed
 * @param goal The goal to shoot at
 *
 * @return A value in [0,1] indicating the percentage of open net the shot is
 *         being taken on. Larger numbers mean a larger percentage of the net is open
 */
double calcShotOpenNetPercentage(const Field &field, const Point &shot_origin,
                                 const Shot &shot, TeamType goal);

/**
 * Function calculates the optimal shot and the corresponding Angle
 * representing the 'open' area of that shot on a given segment to shoot at
 *
 *   Open          Blocked Segment        Open
 * *______X---------------------------X___________________*  <-- reference Segment
 *          .                        .
 *           .                      .
 *            .                    .
 *             .                  .
 *              .                .
 *               .              .
 *                .            .
 *                 .  +----+  .
 *                  . |OBS | .
 *                   .+----+.
 *                    .    .
 *                     .  .
 *                      ..
 *                      X
 *                 Reference Point
 *
 * @param origin The origin of the shot
 * @param segment The segment at which shots are being evaluated on
 * @param robot_obstacles Any Robot (friendly or enemy) that can block the shot
 *
 * @return Shot teturns the optimal Shot (Point and Angle) corresponding to the
 * given parameters
 * @return nullopt if Shot does not exist
 */
std::optional<Shot> calcMostOpenDirectionFromRobotObstacles(
    const Point &origin, const Segment &segment,
    const std::vector<Robot> &robot_obstacles);

/**
 * Function calculates the optimal shot and the corresponding Angle
 * representing the 'open' area of that shot on a given segment to shoot at
 *
 * @param origin The origin of the shot
 * @param segment The segment at which shots are being evaluated on
 * @param obstacles Any obstacle that can block the shot
 *
 * @return the optimal Shot (Point and Angle) corresponding to the
 * given parameters
 * @return nullopt if Shot does not exist
 */
std::optional<Shot> calcMostOpenDirectionFromCircleObstacles(
    const Point &origin, const Segment &segment, const std::vector<Circle> &obstacles);

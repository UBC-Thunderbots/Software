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
 * Finds the best shot on the given goal, and returns the best target to shoot at and
 * the largest open angle interval for the shot (this is the total angle between the
 * obstacles on either side of the shot vector).
 *
 * @param goal_post The goal post of the net by the y-coordinate
 * @param shot_origin The point that the shot will be taken from
 * @param robot_obstacles The locations of any robots on the field that may obstruct
 * the shot. These are treated as circular obstacles, and are usually used to
 * represent robots on the field.
 *
 * @return the best target to shoot at and the largest open angle interval for the
 * shot (this is the total angle between the obstacles on either side of the shot
 * vector). If no shot is possible, returns `std::nullopt`
 */
std::optional<Shot> calcBestShotOnGoal(const Segment &goal_post,
                                       const Point &shot_origin,
                                       const std::vector<Robot> &obstacles);
/**
 * Finds the best shot on the specified goal, and returns the best target to shoot at
 * and the largest open angle interval for the shot (this is the total angle between
 * the obstacles on either side of the shot vector).
 *
 * This function will treat all robots on the field as obstacles, except for the
 * robots_to_ignore
 *
 * @param field The field
 * @param friendly_team The friendly team
 * @param enemy_team The enemy team
 * @param shot_origin The point that the shot will be taken from
 * @param goal The goal to shoot at
 * @param radius The radius for the robot obstacles
 * @param robots_to_ignore The robots to ignore
 *
 * @return the best target to shoot at and the largest open angle interval for the
 * shot (this is the total angle between the obstacles on either side of the shot
 * vector). If no shot can be found, returns std::nullopt
 */

std::optional<Shot> calcBestShotOnGoal(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team, const Point &shot_origin,
                                       TeamType goal,
                                       double radius = ROBOT_MAX_RADIUS_METERS,
                                       const std::vector<Robot> &robots_to_ignore = {});
/**
 * Finds the best shot on the given goal, and returns the best target to shoot at and
 * the largest open angle interval for the shot (this is the total angle between the
 * obstacles on either side of the shot vector).
 *
 * @param goal_post The goal post of the net by the y-coordinate
 * @param shot_origin The point that the shot will be taken from
 * @param obstacles The locations of any circular obstacles on the field that may
 * obstruct the shot. These are usually robots on the field.
 *
 * @return the best target to shoot at and the largest open angle interval for the
 * shot (this is the total angle between the obstacles on either side of the shot
 * vector). If no shot is possible, returns `std::nullopt`
 */
std::optional<Shot> calcBestShotOnGoal(const Segment &goal_post,
                                       const Point &shot_origin,
                                       const std::vector<Circle> &obstacles);

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
 * @param origin : The origin of the shot
 * @param segment : The segment at which shots are being evaluated on
 * @param obstacles : Any obstacle that can block the shot
 *
 * @return Shot : Returns the optimal Shot (Point and Angle) corresponding to the
 * given parameters
 * @return nullopt : A Shot does not exist
 */
std::optional<Shot> calcMostOpenDirectionFromCircleObstacles(
    Point origin, Segment segment, std::vector<Circle> obstacles);

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
 * @param origin : The origin of the shot
 * @param segment : The segment at which shots are being evaluated on
 * @param robot_obstacles : Any Robot (friendly or enemy) that can block the shot
 *
 * @return Shot : Returns the optimal Shot (Point and Angle) corresponding to the
 * given parameters
 * @return nullopt : A Shot does not exist
 */
std::optional<Shot> calcMostOpenDirectionFromRobotObstacles(
    Point origin, Segment segment, std::vector<Robot> robot_obstacles);

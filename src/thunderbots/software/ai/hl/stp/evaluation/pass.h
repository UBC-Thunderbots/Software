/**
 * Declaration of evaluation functions related to passing
 */

#include <optional>

#include "ai/world/ball.h"
#include "ai/world/robot.h"
#include "ai/world/field.h"
#include "geom/point.h"

/**
 * Finds the best place for the given robot to intercept the given ball
 *
 * @param ball The ball to intercept
 * @param field The field on which we want the intercept to occur
 * @param robot The robot that will hopefully intercept the ball
 *
 * @return A pair holding the best place that the robot can move to in order to intercept
 *         the ball, and the duration into the future at which the pass would occur,
 *         relative to the timestamp of the robot. If no possible intercept could be found
 *         within the field bounds, returns std::nullopt
 */
 // TODO: better name for this function?
std::optional<std::pair<Point, Duration>> findBestInterceptForBall(Ball ball, Field field, Robot robot);

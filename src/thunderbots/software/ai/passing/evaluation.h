/**
 * Declaration of evaluation functions for passing
 */

#pragma once

#include <functional>

#include "ai/passing/pass.h"
#include "ai/world/field.h"
#include "ai/world/team.h"
#include "geom/circle.h"
#include "geom/point.h"
#include "geom/rectangle.h"

namespace AI::Passing
{

    // TODO: better name for this function?
    /**
     * Calculate the probability of a friendly robot receiving the given pass
     *
     * Calculate how possible it would be for a robot on the friendly team to receive the
     * given pass, based solely on the robots current position and velocity
     *
     * @param friendly_team The team of robots that might receive the given pass
     * @param pass The pass we want a robot to receive
     *
     * @return A value in [0,1] indicating how likely it would be for a robot on the
     *         friendly team to recive the given pass, with 1 being very likely, 0
     *         being impossible
     */
    double getFriendlyCapability(Team friendly_team, Pass pass);

    /**
     * Calculate how long it would take the given robot to turn to the given orientation
     *
     * @param robot The robot to calculate the rotation time for
     * @param desired_orientation The orientation which we want the robot to be at
     *
     * @return The time required for the given robot to rotate to the given orientation
     */
    Duration getTimeToOrientationForRobot(Robot robot, Angle desired_orientation);

    /**
     * Calculate minimum time it would take for the given robot to reach the given point
     *
     * This is only a rough calculation in order to be as performant as possible
     *
     * @param robot The robot to calculate the time for
     * @param dest The destination that the robot is going to
     *
     * @return The minimum theoretical time it would take the robot to reach the dest
     * point
     */
     // TODO: `Timestamp` should really be `duration` here
    Duration getTimeToPositionForRobot(Robot robot, Point dest);

    /**
     * Calculates the static position quality for a given position on a given field
     *
     * Static position quality prefers good passing points on the field from the
     * perspective of a "real soccer player". For example, passing in front of or towards
     * your own net is less desirable than passing near the enemy's net
     *
     * @param field The field on which to calculate the static position quality
     * @param position The position on the field at which to calculate the quality
     *
     * @return A value in [0,1] representing the quality of the given point on the given
     *         field, with a higher value representing a more desirable position
     */
    double getStaticPositionQuality(const Field& field, const Point& position);

    /**
     * Calculates the value at the given point over a 2D sigmoid over the given rectangle
     *
     * The sigmoid constructed will approach 0 far enough outside the rectangle, and
     * approach 1 far enough within the rectangle. The value on the edge of the rectangle
     * will be 0.5
     *
     * @param rect The rectangle over which to make sigmoid function. The width of the
     *             the rectangle is considered to be in x, and the height in y
     * @param sig_width The length (in either x or y) required to cause the value of the
     *                 sigmoid to go from 0.018 to 0.982
     *
     * @return A value in [0,1], representing the value of the 2D sigmoid function over
     *         the given rectangle at the given point
     */
    double rectangleSigmoid(const Rectangle& rect, const Point& point,
                            const double& sig_width);

    /**
     * Calculates the value at the given point over a 2D sigmoid over the given circle
     *
     * The sigmoid constructed will approach 0 far enough outside the circle, and approach
     * 1 far enough within the circle. The value on the edge of the circle will be 0.5
     *
     * @param circle The circle over which to make sigmoid function
     * @param sig_width The length required to cause the value of the sigmoid to go from
     *                  0.018 to 0.982 across the edge of the circle
     *
     * @return A value in [0,1], representing the value of the 2D sigmoid function over
     *         the given circle at the given point
     */
    double circleSigmoid(const Circle& circle, const Point& point,
                         const double& sig_width);

    /**
     * A sigmoid function with a given offset from 0 and rate of change
     *
     * To flip the sigmoid around (ie. increasing from +v to -v), use a negative sig_width
     *
     * @param v The value to evaluate over the sigmoid
     * @param offset The offset of the center of the  sigmoid from 0
     * @param sig_width The length (in either x or y) required to cause the value of the
     *                 sigmoid to go from 0.018 to 0.982
     *
     * @return A value in [0,1] that is the value of the sigmoid at the value v
     */
    double sigmoid(const double& v, const double& offset, const double& sig_width);

}  // namespace AI::Passing

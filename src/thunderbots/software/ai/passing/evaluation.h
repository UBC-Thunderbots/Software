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
    // TODO: log-sum-exp function with bounds??
    /**
     * Rate pass based on the probability of scoring once we receive the pass
     *
     * @param field The field we are playing on
     * @param enemy_team The enemy team
     * @param pass The pass to rate
     *
     * @return A value in [0,1], with 0 indicating that it's impossible to score off of
     *         the pass, and 1 indicating that it is guaranteed to be able to score off of
     *         the pass
     */
    double ratePassShootScore(Field field, Team enemy_team,
                              AI::Passing::Pass pass);

    // TODO: better name for this function?
    /**
     * Calculates the risk of an enemy robot interfering with a given pass
     *
     * @param enemy_team The team of enemy robots
     * @param pass The pass to rate
     * @return A value in [0,1] indicating the quality of the pass based on the risk
     *         that an enemy interfere with it, with 1 indicating the pass is guaranteed
     *         to run without interference, and 0 indicating that the pass will certainly
     *         be interfered with (and so is very poor)
     */
    double ratePassEnemyRisk(const Team& enemy_team, const Pass& pass);

    /**
     * Calculates the likelihood that the given pass will be intercepted
     *
     * @param enemy_team The team of robots that we're worried about intercepting our pass
     * @param pass The pass we want to get the intercept probability for
     * @return A value in [0,1] indicating the probability that the given pass will be
     *         intercepted by a robot on the given team, with 1 indicating the pass is
     *         guaranteed to be intercepted, and 0 indicating it's impossible for the
     *         pass to be intercepted
     */
    double calculateInterceptRisk(const Team& enemy_team, const Pass& pass);

    /**
     * Calculates the likelihood that the given pass will be intercepted by a given robot
     *
     * @param enemy_robot The robot that might intercept our pass
     * @param pass The pass we want to get the intercept probability for
     * @return A value in [0,1] indicating the probability that the given pass will be
     *         intercepted by the given robot, with 1 indicating the pass is guaranteed to
     *         be intercepted, and 0 indicating it's impossible for the pass to be
     *         intercepted
     */
    double calculateInterceptRisk(Robot enemy_robot, const Pass& pass);


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
    double ratePassFriendlyCapability(const Team& friendly_team, const Pass& pass);

    /**
     * Calculate how long it would take the given robot to turn to the given orientation
     *
     * @param robot The robot to calculate the rotation time for
     * @param desired_orientation The orientation which we want the robot to be at
     *
     * @return The time required for the given robot to rotate to the given orientation
     */
    Duration getTimeToOrientationForRobot(const Robot& robot, const Angle& desired_orientation);

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
    Duration getTimeToPositionForRobot(const Robot& robot, const Point& dest);

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
     * By default this increases from -v to positive v, ie. y = 1 / (1+e^(-x))
     * To flip the sigmoid around (ie. increasing from +v to -v), subtract it from 1
     *
     * When using this function, it is strongly encouraged that you look at it's
     * implementation and go plot it, play around with the numbers a bit to really
     * understand what they're doing.
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

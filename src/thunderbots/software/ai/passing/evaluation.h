/**
 * Declaration of evaluation functions for passing
 */

#pragma once

#include <functional>

#include "ai/passing/pass.h"
#include "ai/world/field.h"
#include "ai/world/team.h"
#include "ai/world/world.h"
#include "util/math_functions.h"

namespace AI::Passing
{
    /**
     * Calculate the quality of a given pass
     *
     * @param world The world in which to rate the pass
     * @param pass The pass to rate
     * @param target_region The area we want to pass to (if there is a specific area,
     *                      set to `std::nullopt` otherwise
     *
     * @return A value in [0,1] representing the quality of the pass, with 1 being an
     *         ideal pass, and 0 being the worst pass possible
     */
    double ratePass(const World& world, const AI::Passing::Pass& pass,
                    const std::optional<Rectangle>& target_region);

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
    double ratePassShootScore(const Field& field, const Team& enemy_team,
                              const AI::Passing::Pass& pass);

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
     *         friendly team to recieve the given pass, with 1 being very likely, 0
     *         being impossible
     */
    double ratePassFriendlyCapability(const Team& friendly_team, const Pass& pass);

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

}  // namespace AI::Passing

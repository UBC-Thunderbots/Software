#pragma once

#include <functional>

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/parameters.pb.h"
#include "software/ai/passing/pass.h"
#include "software/math/math_functions.h"
#include "software/util/make_enum/make_enum.h"
#include "software/world/field.h"
#include "software/world/team.h"
#include "software/world/world.h"

/**
 * Calculate the quality of a given pass
 *
 * @param world The world in which to rate the pass
 * @param pass The pass to rate
 * @param passing_config The passing config used for tuning
 *
 * @return A value in [0,1] representing the quality of the pass, with 1 being an
 *         ideal pass, and 0 being the worst pass possible
 */
double ratePass(const World& world, const Pass& pass,
                const TbotsProto::PassingConfig& passing_config);

/**
 * Calculate the quality of a given pass accounting for the zone it's in
 *
 * @param world The world in which to rate the pass
 * @param pass The pass to rate
 * @param zone The zone this pass is constrained to
 * @param passing_config The passing config used for tuning
 *
 * @return A value in [0,1] representing the quality of the pass, with 1 being an
 *         ideal pass, and 0 being the worst pass possible
 */
double ratePass(const World& world, const Pass& pass, const Rectangle& zone,
                const TbotsProto::PassingConfig& passing_config);

/**
 * Calculate the quality of a given zone
 *
 * @param field The field on which to rate the zone
 * @param enemy_team The enemy team
 * @param zone The zone to rate
 * @param ball_position The position of the ball
 * @param passing_config The passing config used for tuning
 *
 * @return A value in [0,1] representing the quality of the zone, with 1 being a
 *         great zone to send a cherry picker to, and 0 being a zone to avoid.
 */
double rateZone(const Field& field, const Team& enemy_team, const Rectangle& zone,
                const Point& ball_position,
                const TbotsProto::PassingConfig& passing_config);

/**
 * Rate a pass based on the quality of the receiving position
 *
 * @param world The world in which to rate the pass
 * @param pass The pass to rate
 * @param passing_config The passing config used for tuning
 * @return A value in [0,1] representing the quality of the pass receiving
 * position, with 1 being indicating that the receiving position is ideal, and 0
 * indicating that the pass will likely not be received.
 */
double rateReceivingPosition(const World& world, const Pass& pass,
                             const TbotsProto::PassingConfig& passing_config);

/**
 * Rate a point to shoot on enemy goal from
 *
 * @param field The field we are playing on
 * @param enemy_team The enemy team
 * @param shot_origin The point to shoot from
 * @param passing_config The passing config used for tuning
 * @return A value in [0,1] representing the quality of the shot, with 1 being
 *       an ideal shot, and 0 being a shot that will most likely be blocked.
 */
double rateShot(const Point& shot_origin, const Field& field, const Team& enemy_team,
                const TbotsProto::PassingConfig& passing_config);

/**
 * Rate pass based on the probability of scoring once we receive the pass
 *
 * @param field The field we are playing on
 * @param enemy_team The enemy team
 * @param pass The pass to rate
 * @param passing_config The passing config used for tuning
 *
 * @return A value in [min_pass_shoot_score,1], with min_pass_shoot_score indicating that
 * it's impossible to score off of the pass, and 1 indicating that it is guaranteed to be
 * able to score off of the pass
 */
double ratePassShootScore(const Field& field, const Team& enemy_team, const Pass& pass,
                          const TbotsProto::PassingConfig& passing_config);

/**
 * Calculates the risk of an enemy robot interfering with a given pass
 *
 * @param enemy_team The team of enemy robots
 * @param pass The pass to rate
 * @param passing_config The passing config used for tuning
 * @return A value in [0,1] indicating the quality of the pass based on the risk
 *         that an enemy interfere with it, with 1 indicating the pass is guaranteed
 *         to run without interference, and 0 indicating that the pass will certainly
 *         be interfered with (and so is very poor)
 */
double ratePassEnemyRisk(const Team& enemy_team, const Pass& pass,
                         const TbotsProto::PassingConfig& passing_config);

/**
 * Rate the pass based on if it moves the ball up the field or not
 * Passes moving the ball up the field are rated higher
 *
 * @param pass The pass to rate
 * @param passing_config The passing config used for tuning
 * @return A value in [0,1] indicating the quality of the pass, where
 *        1 indicates the pass is ideal and 0 indicates the pass is bad as
 *        it passes back toward our friendly half.
 */
double ratePassForwardQuality(const Pass& pass,
                              const TbotsProto::PassingConfig& passing_config);

/**
 * Encourage passes that are not too close to the passer
 * @param pass The pass to rate
 * @param passing_config The passing config used for tuning
 * @return A value in [0,1] indicating the quality of the pass, where
 *        1 indicates the pass is ideal and 0 indicates the pass is bad as
 *        it is too close to the passer.
 */
double ratePassNotTooClose(const Pass& pass,
                           const TbotsProto::PassingConfig& passing_config);

/**
 * Calculates the likelihood that the given pass will be intercepted
 *
 * @param enemy_team The team of robots that we're worried about intercepting our pass
 * @param pass The pass we want to get the intercept probability for
 * @param passing_config The passing config used for tuning
 * @return A value in [0,1] indicating the probability that the given pass will be
 *         intercepted by a robot on the given team, with 1 indicating the pass is
 *         guaranteed to be intercepted, and 0 indicating it's impossible for the
 *         pass to be intercepted
 */
double calculateInterceptRisk(const Team& enemy_team, const Pass& pass,
                              const TbotsProto::PassingConfig& passing_config);

/**
 * Calculates the likelihood that the given pass will be intercepted by a given robot
 *
 * @param enemy_robot The robot that might intercept our pass
 * @param pass The pass we want to get the intercept probability for
 * @param passing_config The passing config used for tuning
 * @return A value in [0,1] indicating the probability that the given pass will be
 *         intercepted by the given robot, with 1 indicating the pass is guaranteed to
 *         be intercepted, and 0 indicating it's impossible for the pass to be
 *         intercepted
 */
double calculateInterceptRisk(const Robot& enemy_robot, const Pass& pass,
                              const TbotsProto::PassingConfig& passing_config);


/**
 * Calculate the probability of a friendly robot receiving the given pass
 *
 * Calculate how possible it would be for a robot on the friendly team to receive the
 * given pass, based solely on the robots current position and velocity
 *
 * @param friendly_team The team of robots that might receive the given pass
 * @param pass The pass we want a robot to receive
 * @param passing_config The passing config used for tuning
 *
 * @return A value in [0,1] indicating how likely it would be for a robot on the
 *         friendly team to receive the given pass, with 1 being very likely, 0
 *         being impossible
 */
double ratePassFriendlyCapability(const Team& friendly_team, const Pass& pass,
                                  const TbotsProto::PassingConfig& passing_config);

/**
 * Calculates the static position quality for a given position on a given field
 *
 * Static position quality prefers good passing points on the field from the
 * perspective of a "real soccer player". For example, passing in front of or towards
 * your own net is less desirable than passing near the enemy's net
 *
 * @param field The field on which to calculate the static position quality
 * @param position The position on the field at which to calculate the quality
 * @param passing_config The passing config used for tuning
 *
 * @return A value in [0,1] representing the quality of the given point on the given
 *         field, with a higher value representing a more desirable position
 */
double getStaticPositionQuality(const Field& field, const Point& position,
                                const TbotsProto::PassingConfig& passing_config);

/**
 * Returns a function that increases as the point approaches enemy robots.
 *
 * @param point a Point
 * @param enemy_team the enemy team
 * @param passing_config The passing config used for tuning
 * @return a measure of how close the point is to one or more enemy robots
 */
double calculateProximityRisk(const Point& point, const Team& enemy_team,
                              const TbotsProto::PassingConfig& passing_config);

/**
 * Calculate the quality of a position for staying away from enemy robots
 * while trying to create opportunity for passes and shots.
 *
 * @param keep_away_position The position to rate
 * @param world The world in which to rate the pass
 * @param best_pass_so_far The best pass so far used for rating passing opportunity of
 * the keep away position
 * @param dribbling_bounds The bounds of the area the robot can dribble in
 * @param passing_config The passing config used for tuning
 * @return A value in [0,1] representing the quality of the passer position, with 1
 *        being an ideal position to pass from, and 0 being a poor position to pass from.
 */
double rateKeepAwayPosition(const Point& keep_away_position, const World& world,
                            const Pass& best_pass_so_far,
                            const Rectangle& dribbling_bounds,
                            const TbotsProto::PassingConfig& passing_config);

/**
 * TODO (NIMA)
 * @param rating
 * @param min
 * @param max
 * @return
 */
double scaleNormalizedRating(double rating, double min, double max);

/**
 * Sample passes at different points on the field and rate them, similar to ratePass, to
 * be visualized in thunderscope
 *
 * The cost functions used to rate the pass depends on passing_config.cost_vis_config
 * configuration, this can be controlled from thunderscope The number of points sampled is
 * also controlled from there.
 *
 * The sampled values are sent over protobuf to thunderscope as a CostVisualization
 * message. These values are eventually visualized in thunderscope in the cost_vis widget
 *
 * @param world The world in which to sample passes
 * @param passing_config The passing config used for tuning
 * @param best_pass_so_far The best pass so far used for sampling best passer position
 *
 */
void samplePassesForVisualization(
    const World& world, const TbotsProto::PassingConfig& passing_config,
    const std::optional<Pass>& best_pass_so_far = std::nullopt);

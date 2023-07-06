#pragma once

#include <functional>

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/parameters.pb.h"
#include "software/ai/passing/pass.h"
#include "software/math/math_functions.h"
#include "software/geom/algorithms/distance.h"
#include "software/util/make_enum/make_enum.h"
#include "software/world/field.h"
#include "software/world/team.h"
#include "software/world/world.h"

/**
 * Calculate the quality of a given pass
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
                TbotsProto::PassingConfig passing_config);

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
                const Point& ball_position, TbotsProto::PassingConfig passing_config);

/**
 * Rate pass based on the probability of scoring once we receive the pass
 *
 * @param field The field we are playing on
 * @param enemy_team The enemy team
 * @param pass The pass to rate
 * @param passing_config The passing config used for tuning
 *
 * @return A value in [0,1], with 0 indicating that it's impossible to score off of
 *         the pass, and 1 indicating that it is guaranteed to be able to score off of
 *         the pass
 */
double ratePassShootScore(const Field& field, const Team& enemy_team, const Pass& pass,
                          TbotsProto::PassingConfig passing_config);

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
                         const Duration& enemy_reaction_time,
                         double enemy_proximity_importance);

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
                              const Duration& enemy_reaction_time);

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
                              const Duration& enemy_reaction_time);


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
                                  TbotsProto::PassingConfig passing_config);

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
                                TbotsProto::PassingConfig passing_config);

/**
 * Returns a function that increases as the point approaches enemy robots.
 *
 * @param point a Point
 * @param enemy_team the enemy team
 * @param enemy_proximity_importance essentially a scaling factor for the result
 * @return a measure of how close the point is to one or more enemy robots
 */
double calculateProximityRisk(const Point& point, const Team& enemy_team,
                              double enemy_proximity_importance);


double getPassDistanceQuality(const Pass &pass);

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
 */
void samplePassesForVisualization(const World& world,
                                  const TbotsProto::PassingConfig& passing_config);

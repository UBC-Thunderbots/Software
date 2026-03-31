#pragma once
#include <zlib.h>

#include "pass.h"
#include "software/world/world.h"

/*
 * This class is responsible for collecting the features and the results
 * for each pass considered by the Pass Generator
 *
 * These will tell the model what a "bad" pass is:
 * We can't make any determinations on if the pass is good, since
 * we don't know the game state outcome yet. But, we know which
 * passes we'd like to exclude / weigh down
 *
 * These are the features we want to collect:
 *
 * Pass Receive Position x and y
 * Ball Position x and y
 * Positions of friendly and enemy robots, x and y
 * The Score we assigned to the pass
 *
 * These will all be sent within the PassFeatures proto to be consumed later
 *
 *
 * This is how we determine the final result for each pass:
 *
 * Definitely Bad: These are passes that either break an SSL rule
 *                 or passes we just want to avoid completely
 *                 they get a very negative score.
 *
 *                 Eg: a pass into the enemy defense area (illegal)
 *                     a pass that the receiver cannot physically receive
 *
 * Bad, Slightly Bad, : These are additive scores for passes
 * Very Slightly Bad    that are not illegal, but we would want to avoid.
 *                      These are heuristics based and so have some bias
 *                      but that's unavoidable.
 *                      We make the score more negative the worse the pass is.
 *
 *                      Eg: a pass that the receive would reached just in time (risky)
 *
 * Everything else gets a score of 0, so a neutral pass.
 *
 *
 * A lot of the logic is ported over from existing cost functions. However,
 * we want to exclude the more "subjective" metrics when scoring passes. Specifically:
 *
 * 1. getStaticPositionQuality
 *    - exclude near_friendly_goal_quality
 *    - include in_enemy_defense_area_quality, on_field_quality
 * 2. ratePassNotTooClose -> exclude completely
 * 3. ratePassFriendlyCapability -> use completely
 * 4. ratePassForwardQuality -> use completely
 * 5. ratePassEnemyRisk
 *    - exclude enemy_receiver_proximity_risk
 *    - include intercept_risk, but risk -> score translation is different
 * 6. ratePassShootScore -> check if there's shot on goal the same
 */
class PassFeatureCollector
{
   public:
    PassFeatureCollector();

    /**
     * Gets the features and score for the given pass within the given world state
     * And sends out a proto with the feature info
     *
     * @param pass the pass to get features from
     * @param world the current world state
     * @param passing_config the static config for passes, to compare against for scoring
     */
    void logPassFeatures(const Pass& pass, const World& world,
                         const TbotsProto::PassingConfig& passing_config);

   private:
    /**
     * Gets the score for the given pass in the given world state
     * Compares against the passing config for scoring
     * @param pass the pass to score
     * @param world the current world state
     * @param passing_config the config containing values to compare against
     * @return a numerical score to quantify if the pass is bad or neutral
     */
    double getPassScore(const Pass& pass, const World& world,
                        const TbotsProto::PassingConfig passing_config);

    double getEnemyInterceptTimeDelta(const Robot& enemy_robot, const Pass& pass,
                                      const TbotsProto::PassingConfig& passing_config);

    // Labels for bad passes
    static constexpr double DEFINITELY_BAD_SCORE    = -100;
    static constexpr double BAD_SCORE               = -3;
    static constexpr double SLIGHTLY_BAD_SCORE      = -1;
    static constexpr double VERY_SLIGHTLY_BAD_SCORE = -0.1;
    static constexpr double NEUTRAL_SCORE           = 0;

    // Translating Sigmoid scores in [0, 1] into heuristics
    static constexpr double BAD_SIGMOID_SCORE     = 0.25;
    static constexpr double NEUTRAL_SIGMOID_SCORE = 0.5;
    static constexpr double GOOD_SIGMOID_SCORE    = 0.75;

    // If the enemy can get in the ball's path before the ball within n seconds, it's
    // risky
    static constexpr double RISKY_INTERCEPT_DELTA = 0.75;
};

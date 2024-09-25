#pragma once

#include "software/ai/rl/attacker/attacker_mdp_action.h"
#include "software/ai/rl/attacker/attacker_mdp_state.h"
#include "software/ai/rl/feature_extractor.hpp"

/**
 * FeatureExtractor that extracts the values of features from a AttackerMdpState.
 */
class AttackerMdpFeatureExtractor
    : public FeatureExtractor<AttackerMdpState, AttackerMdpAction>
{
   public:
    explicit AttackerMdpFeatureExtractor();

    /**
     * Extracts the value of a feature representing the ball's x-distance
     * to the enemy goal line.
     *
     * @param state the state to extract the feature from
     *
     * @return the value of the feature
     */
    static double ballXPositionFeature(const AttackerMdpState& state);

    /**
     * Extracts the value of a feature representing the rating of the current
     * best pass the Attacker agent can take.
     *
     * @param state the state to extract the feature from
     *
     * @return the value of the feature
     */
    static double bestPassRatingFeature(const AttackerMdpState& state);

    /**
     * Extracts the value of a feature representing the open angle of the current
     * best shot on goal the Attacker agent can take.
     *
     * @param state the state to extract the feature from
     *
     * @return the value of the feature
     */
    static double bestShotOpenAngleFeature(const AttackerMdpState& state);

    /**
     * Extracts the value of a feature representing how many enemy threats
     * are nearby the ball and could potentially steal it.
     *
     * @param state the state to extract the feature from
     *
     * @return the value of the feature
     */
    static double nearbyEnemyThreatsFeature(const AttackerMdpState& state);

    /**
     * Extracts the value of a feature representing the number of friendly
     * robots on the field. The fewer robots on the field, the larger the
     * feature value (i.e. this feature is more important when we are down
     * robots).
     *
     * @param state the state to extract the feature from
     *
     * @return the value of the feature
     */
    static double numFriendlyRobotsFeature(const AttackerMdpState& state);

    /**
     * Extracts the value of a feature representing the number of enemy
     * robots on the field. The fewer robots on the field, the larger the
     * feature value (i.e. this feature is more important when they are down
     * robots).
     *
     * @param state the state to extract the feature from
     *
     * @return the value of the feature
     */
    static double numEnemyRobotsFeature(const AttackerMdpState& state);

   protected:
    // Factor to multiply the open shot angle by in bestShotOpenAngleFeature
    static constexpr double SHOT_OPEN_ANGLE_FACTOR = 0.75;
};

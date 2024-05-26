#pragma once

#include "software/ai/evaluation/q_learning/attacker_mdp_action.h"
#include "software/ai/evaluation/q_learning/feature_extractor.hpp"

/**
 * State representation of the Markov decision process (MDP) modelling
 * the Attacker agent's gameplay decision making.
 */
struct AttackerMdpState
{
    WorldPtr world_ptr;
    std::shared_ptr<Strategy> strategy;
};

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
     * Extracts the value of a feature representing the ball's y-distance
     * to the halfway line (which runs along the width of the field and through
     * the center of the field).
     *
     * @param state the state to extract the feature from
     *
     * @return the value of the feature
     */
    static double ballYPositionFeature(const AttackerMdpState& state);

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
};

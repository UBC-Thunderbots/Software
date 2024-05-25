#pragma once

#include "software/ai/evaluation/q_learning/attacker_mdp_action.h"
#include "software/ai/evaluation/q_learning/feature_extractor.hpp"

struct AttackerMdpState
{
    WorldPtr world_ptr;
    std::shared_ptr<Strategy> strategy;
};

class AttackerMdpFeatureExtractor
    : public FeatureExtractor<AttackerMdpState, AttackerMdpAction>
{
   public:
    explicit AttackerMdpFeatureExtractor();

    static double ballXPositionFeature(const AttackerMdpState& state);

    static double ballYPositionFeature(const AttackerMdpState& state);

    static double bestPassRatingFeature(const AttackerMdpState& state);
};

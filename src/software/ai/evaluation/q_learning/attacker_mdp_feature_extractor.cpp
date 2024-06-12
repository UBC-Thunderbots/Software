#include "software/ai/evaluation/q_learning/attacker_mdp_feature_extractor.h"

#include <math.h>

#include "software/ai/evaluation/possession.h"

AttackerMdpFeatureExtractor::AttackerMdpFeatureExtractor()
    : FeatureExtractor({ballXPositionFeature, bestPassRatingFeature, bestShotOpenAngleFeature})
{
}

double AttackerMdpFeatureExtractor::ballXPositionFeature(const AttackerMdpState& state)
{
    const double x_pos = state.world_ptr->ball()
        .estimateFutureState(Duration::fromSeconds(1.0)).position().x();
    const double field_x_length   = state.world_ptr->field().totalXLength();
    const double normalized_x_pos = std::max(x_pos, 0.0) / (field_x_length / 2);
    return normalized_x_pos;
}

double AttackerMdpFeatureExtractor::bestPassRatingFeature(const AttackerMdpState& state)
{
    return state.strategy->getBestPass().rating;
}

double AttackerMdpFeatureExtractor::bestShotOpenAngleFeature(
    const AttackerMdpState& state)
{
    std::optional<Robot> attacker = getRobotWithEffectiveBallPossession(
        state.world_ptr->friendlyTeam(), state.world_ptr->ball(),
        state.world_ptr->field());

    if (attacker)
    {
        std::optional<Shot> best_shot = state.strategy->getBestShot(*attacker);
        if (best_shot)
        {
            return best_shot->getOpenAngle().toRadians() / (M_PI / 4);
        }
    }

    return 0;
}

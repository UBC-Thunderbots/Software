#include "software/ai/evaluation/q_learning/attacker_mdp_feature_extractor.h"

#include <math.h>
#include "software/ai/evaluation/possession.h"

AttackerMdpFeatureExtractor::AttackerMdpFeatureExtractor()
    : FeatureExtractor({ballXPositionFeature, ballYPositionFeature, bestPassRatingFeature,
                        bestShotOpenAngleFeature})
{
}

double AttackerMdpFeatureExtractor::ballXPositionFeature(const AttackerMdpState& state)
{
    const double x_pos            = state.world_ptr->ball().position().x();
    const double field_x_length   = state.world_ptr->field().totalXLength();
    const double normalized_x_pos = (x_pos + field_x_length / 2) / field_x_length;
    return normalized_x_pos;
}

double AttackerMdpFeatureExtractor::ballYPositionFeature(const AttackerMdpState& state)
{
    const double y_pos            = state.world_ptr->ball().position().y();
    const double field_y_length   = state.world_ptr->field().totalYLength();
    const double normalized_y_pos = (y_pos + field_y_length / 2) / field_y_length;
    return normalized_y_pos;
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
            return best_shot->getOpenAngle().toRadians() / M_PI;
        }
    }

    return 0;
}

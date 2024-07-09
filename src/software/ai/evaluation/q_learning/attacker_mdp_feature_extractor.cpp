#include "software/ai/evaluation/q_learning/attacker_mdp_feature_extractor.h"

#include <math.h>

#include "shared/constants.h"
#include "software/ai/evaluation/possession.h"
#include "software/math/math_functions.h"

AttackerMdpFeatureExtractor::AttackerMdpFeatureExtractor()
    : FeatureExtractor({{"ball_x_position", ballXPositionFeature},
                        {"best_pass_rating", bestPassRatingFeature},
                        {"best_shot_open_angle", bestShotOpenAngleFeature},
                        {"num_friendly_robots", numFriendlyRobotsFeature},
                        {"num_enemy_robots", numEnemyRobotsFeature}})
{
}

double AttackerMdpFeatureExtractor::ballXPositionFeature(const AttackerMdpState& state)
{
    const double x_pos            = state.world_ptr->ball().position().x();
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
            return best_shot->getOpenAngle().toDegrees() * SHOT_OPEN_ANGLE_FACTOR;
        }
    }

    return 0;
}

double AttackerMdpFeatureExtractor::numFriendlyRobotsFeature(
    const AttackerMdpState& state)
{
    const size_t num_robots = state.world_ptr->friendlyTeam().numRobots();
    return normalizeValueToRange(static_cast<double>(num_robots), 0.0,
                                 static_cast<double>(DIV_B_NUM_ROBOTS), 1.0, 0.0);
}

double AttackerMdpFeatureExtractor::numEnemyRobotsFeature(const AttackerMdpState& state)
{
    const size_t num_robots = state.world_ptr->enemyTeam().numRobots();
    return normalizeValueToRange(static_cast<double>(num_robots), 0.0,
                                 static_cast<double>(DIV_B_NUM_ROBOTS), 1.0, 0.0);
}

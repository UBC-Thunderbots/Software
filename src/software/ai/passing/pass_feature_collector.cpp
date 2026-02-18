
#include "pass_feature_collector.h"

#include <utility>

#include "cost_function.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

PassFeatureCollector::PassFeatureCollector() {}

void PassFeatureCollector::logPassFeatures(const Pass& pass, const World& world,
                                           TbotsProto::PassingConfig passing_config) const
{
    double score = getPassScore(pass, world, passing_config);

    LOG(VISUALIZE) << *createPassFeaturesProto(pass, world, score);
}

double PassFeatureCollector::getPassScore(const Pass& pass, const World& world,
                                          TbotsProto::PassingConfig passing_config)
{
    // if the pass has 0 speed
    if (pass.speed() == 0)
        return DEFINITELY_BAD_SCORE;

    // if there are no receivers on the friendly team
    if (world.friendlyTeam().getAllRobots().size() <= 0)
        return DEFINITELY_BAD_SCORE;

    // if pass is not far enough from the passer
    auto passLength = (pass.receiverPoint() - pass.passerPoint()).length();
    if (passLength < passing_config.receiver_ideal_min_distance_meters())
        return DEFINITELY_BAD_SCORE;

    const auto field = world.field();

    // Make a slightly smaller field, and positive weight values in this reduced field
    // if pass receive point not in the reduced field boundaries
    if (const auto reduced_size_field = getReducedField(field, passing_config);
        !contains(reduced_size_field, pass.receiverPoint()))
        return DEFINITELY_BAD_SCORE;

    // if pass receive point is in the enemy defense area (illegal)
    if (!contains(field.enemyDefenseArea(), pass.receiverPoint()))
        return DEFINITELY_BAD_SCORE;

    auto friendlyReceiveCapabilitySigmoid =
        ratePassFriendlyCapability(world.friendlyTeam(), pass, passing_config);

    // robot arrives / turns too late
    if (friendlyReceiveCapabilitySigmoid < 0.5)
        return DEFINITELY_BAD_SCORE;

    auto passForwardSigmoid = ratePassForwardQuality(pass, passing_config);
    if (passForwardSigmoid < 0.25)
        return DEFINITELY_BAD_SCORE;
    if (passForwardSigmoid < 0.5)
        return SLIGHTLY_BAD_SCORE;

    auto shot_opt = calcBestShotOnGoal(
        Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg()), pass.receiverPoint(),
        world.enemyTeam().getAllRobots(), TeamType::ENEMY);

    // if there's no shot on goal, rate it very slightly bad
    if (!shot_opt)
        return VERY_SLIGHTLY_BAD_SCORE;

    return NEUTRAL_SCORE;
}

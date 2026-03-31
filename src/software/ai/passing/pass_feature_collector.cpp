
#include "pass_feature_collector.h"

#include <utility>

#include "cost_function.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/logger/logger.h"

PassFeatureCollector::PassFeatureCollector() {}

void PassFeatureCollector::logPassFeatures(
    const Pass& pass, const World& world, const TbotsProto::PassingConfig& passing_config)
{
    double score = getPassScore(pass, world, passing_config);

    LOG(VISUALIZE) << *createPassFeaturesProto(pass, world, score);
}

double PassFeatureCollector::getEnemyInterceptTimeDelta(
    const Robot& enemy_robot, const Pass& pass,
    const TbotsProto::PassingConfig& passing_config)
{
    Point closest_interception_point = closestPoint(
        enemy_robot.position(), Segment(pass.passerPoint(), pass.receiverPoint()));

    double time_to_interception_s =
        getEnemyTimeToInterceptPoint(enemy_robot, pass, closest_interception_point);

    Duration ball_time_to_interception_point =
        Duration::fromSeconds(distance(pass.passerPoint(), closest_interception_point) /
                              pass.speed()) +
        Duration::fromSeconds(passing_config.pass_delay_sec());

    return time_to_interception_s - ball_time_to_interception_point.toSeconds();
}

double PassFeatureCollector::getPassScore(const Pass& pass, const World& world,
                                          const TbotsProto::PassingConfig passing_config)
{
    double score = NEUTRAL_SCORE;

    // if the pass has 0 speed
    if (pass.speed() == 0)
        return DEFINITELY_BAD_SCORE;

    // if there are no receivers on the friendly team
    if (world.friendlyTeam().getAllRobots().size() <= 0)
        return DEFINITELY_BAD_SCORE;

    const Field& field = world.field();

    // in_enemy_defense_area_quality -> if pass receive point is in the enemy defense area
    // (illegal)
    if (contains(field.enemyDefenseArea(), pass.receiverPoint()))
        return DEFINITELY_BAD_SCORE;

    // on_field_quality -> Make a slightly smaller field, and check
    // if pass receive point not in the reduced field boundaries
    if (!contains(getReducedField(field, passing_config), pass.receiverPoint()))
        score += BAD_SCORE;

    // how well the receiver can receive the ball
    double friendlyReceiveCapabilitySigmoid =
        ratePassFriendlyCapability(world.friendlyTeam(), pass, passing_config);

    // robot arrives / turns too late
    if (friendlyReceiveCapabilitySigmoid < BAD_SIGMOID_SCORE)
        return DEFINITELY_BAD_SCORE;
    else if (friendlyReceiveCapabilitySigmoid < NEUTRAL_SIGMOID_SCORE)
        score += BAD_SCORE;

    // how well any enemy can intercept the ball
    for (const auto& robot : world.enemyTeam().getAllRobots())
    {
        double intercept_time_delta =
            getEnemyInterceptTimeDelta(robot, pass, passing_config);

        // if any enemy can intercept successfully
        if (intercept_time_delta <= 0)
            return DEFINITELY_BAD_SCORE;
        // if the interception is close (risky)
        else if (intercept_time_delta < RISKY_INTERCEPT_DELTA)
            score += SLIGHTLY_BAD_SCORE;
    }

    double passForwardSigmoid = ratePassForwardQuality(pass, passing_config);
    if (passForwardSigmoid < BAD_SIGMOID_SCORE)
        score += BAD_SCORE;

    auto shot_opt = calcBestShotOnGoal(
        Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg()), pass.receiverPoint(),
        world.enemyTeam().getAllRobots(), TeamType::ENEMY);

    // if there's no shot on goal, rate it very slightly bad
    if (!shot_opt)
        score += VERY_SLIGHTLY_BAD_SCORE;

    return score;
}

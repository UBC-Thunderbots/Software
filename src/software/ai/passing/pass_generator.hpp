#pragma once

#include <algorithm>
#include <chrono>
#include <mutex>
#include <numeric>
#include <random>
#include <thread>

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/parameters.pb.h"
#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass.h"
#include "software/ai/passing/pass_evaluation.hpp"
#include "software/ai/passing/pass_with_rating.h"
#include "software/logger/logger.h"
#include "software/optimization/gradient_descent_optimizer.hpp"
#include "software/time/timestamp.h"
#include "software/world/world.h"

// The random seed to initialize the random number generator
static const int PASS_GENERATOR_SEED = 14;

template <class ZoneEnum>
using ZonePassMap = std::unordered_map<ZoneEnum, PassWithRating>;

/**
 * This class is responsible for generating passes for us to perform
 */
template <class ZoneEnum>
class PassGenerator
{
    static_assert(std::is_enum<ZoneEnum>::value,
                  "PassGenerator: ZoneEnum must be a zone id enum");

   public:
    /**
     * Creates a new PassGenerator with the given pitch_division.
     *
     * The PassGenerator will use this pitch division to guide initial random samples
     * in each zone after the pitch has been divided.
     *
     * @param pitch_division The pitch division to use when looking for passes
     */
    explicit PassGenerator(
        std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division,
        TbotsProto::PassingConfig passing_config);

    /**
     * Creates a PassEvaluation given a world and a field pitch division.
     *
     * NOTE: If we want to run our AI at 30hz, it gives us 1/30 = 33ms between ticks.
     * This function needs to run in less than 1/3 of that time (< 10ms) to allow
     * for other modules in our AI to have enough time to run.
     *
     * Passes are evaluated on the provided world. If the evaluation takes longer than
     * the time between two vision frames, we will be evaluating on an outdated world.
     *
     * Because of this, it is extremely important that the pass generator runs fast
     * enough. It is recommended that all testing of things involving the PassGenerator
     * be done with executables built in "Release" in order to maximize performance
     * ("Release" can be 2-10x faster then "Debug").
     *
     * @param world The world to compute the pass evaluation on
     *
     * @return The best currently known pass and the rating of that pass (in [0-1])
     */
    PassEvaluation<ZoneEnum> generatePassEvaluation(const World& world);


   private:
    // Weights used to normalize the parameters that we pass to GradientDescent
    // (see the GradientDescent documentation for details)
    // These weights are *very* roughly the step that gradient descent will take
    // in each respective dimension for a single iteration. They are tuned to
    // ensure passes converge as fast as possible, but are also as stable as
    // possible
    static constexpr double PASS_SPACE_WEIGHT                          = 0.1;
    static constexpr double PASS_SPEED_WEIGHT                          = 0.01;
    std::array<double, NUM_PARAMS_TO_OPTIMIZE> optimizer_param_weights = {
        PASS_SPACE_WEIGHT, PASS_SPACE_WEIGHT, PASS_SPEED_WEIGHT};

    /**
     * Randomly samples a receive point across every zone and assigns a random
     * speed to each pass.
     *
     * @returns a mapping of the Zone Id to the sampled pass
     */
    ZonePassMap<ZoneEnum> samplePasses(const World& world);

    /**
     * Generates a new pass to each zone's centre and rates them to pass it to
     * thunderscope
     */
    void sampleZoneCentrePasses(const World& world);

    /**
     * Given a map of passes, runs a gradient descent optimizer to find
     * better passes.
     *
     * @param The world
     * @param The passes to be optimized mapped to the zone
     * @returns a mapping of the Zone id to the optimized pass
     */
    ZonePassMap<ZoneEnum> optimizePasses(const World& world,
                                         const ZonePassMap<ZoneEnum>& initial_passes);

    /**
     * Re-evaluates ratePass on the previous world's passes and keeps the better pass
     * w/ the higher score in current_best_passes_;
     *
     * @param The world
     * @param optimized_passes The optimized_passes to update our internal cached
     * passes with.
     */
    void updatePasses(const World& world, const ZonePassMap<ZoneEnum>& optimized_passes);

    // All the passes that we are currently trying to optimize in gradient descent
    ZonePassMap<ZoneEnum> current_best_passes_;

    // The optimizer we're using to find passes
    GradientDescentOptimizer<NUM_PARAMS_TO_OPTIMIZE> optimizer_;

    // Pitch division
    std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division_;

    // Passing configuration
    TbotsProto::PassingConfig passing_config_;

    // A random number generator for use across the class
    std::mt19937 random_num_gen_;
};
template <class ZoneEnum>
PassGenerator<ZoneEnum>::PassGenerator(
    std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division,
    TbotsProto::PassingConfig passing_config)
    : optimizer_(optimizer_param_weights),
      pitch_division_(pitch_division),
      passing_config_(passing_config),
      random_num_gen_(PASS_GENERATOR_SEED)
{
}

template <class ZoneEnum>
PassEvaluation<ZoneEnum> PassGenerator<ZoneEnum>::generatePassEvaluation(
    const World& world)
{
    // Generate sample passes for cost visualization
    if (passing_config_.generate_sample_passes())
    {
        sampleZoneCentrePasses(world);
    }

    auto generated_passes = samplePasses(world);
    if (current_best_passes_.empty())
    {
        current_best_passes_ = generated_passes;
    }
    auto optimized_passes = optimizePasses(world, generated_passes);

    updatePasses(world, optimized_passes);


    std::vector<PassWithRating> passes;
    passes.reserve(current_best_passes_.size());

    for (auto zone_and_pass : current_best_passes_)
    {
        passes.push_back(zone_and_pass.second);
    }

    LOG(VISUALIZE) << *createPassVisualization(passes);

    return PassEvaluation<ZoneEnum>(pitch_division_, current_best_passes_,
                                    passing_config_, world.getMostRecentTimestamp());
}

template <class ZoneEnum>
ZonePassMap<ZoneEnum> PassGenerator<ZoneEnum>::samplePasses(const World& world)
{
    std::uniform_real_distribution speed_distribution(
        passing_config_.min_pass_speed_m_per_s(),
        passing_config_.max_pass_speed_m_per_s());

    ZonePassMap<ZoneEnum> passes;

    // Randomly sample a pass in each zone
    for (ZoneEnum zone_id : pitch_division_->getAllZoneIds())
    {
        auto zone = pitch_division_->getZone(zone_id);

        std::uniform_real_distribution x_distribution(zone.xMin(), zone.xMax());
        std::uniform_real_distribution y_distribution(zone.yMin(), zone.yMax());

        auto pass =
            Pass(world.ball().position(),
                 Point(x_distribution(random_num_gen_), y_distribution(random_num_gen_)),
                 speed_distribution(random_num_gen_));

        passes.emplace(
            zone_id,
            PassWithRating{pass, ratePass(world, pass, pitch_division_->getZone(zone_id),
                                          passing_config_)});
    }

    return passes;
}

template <class ZoneEnum>
void PassGenerator<ZoneEnum>::sampleZoneCentrePasses(const World& world)
{
    // this decides the numbers of rows and cols we visualize
    // TODO: move to contants file
    int NUM_ROWS  = 3;
    int NUM_COLS  = 6;
    double width  = world.field().xLength() / NUM_COLS;
    double height = world.field().yLength() / NUM_ROWS;

    std::unordered_map<std::string, std::vector<double>> func_to_costs;
    std::vector<double> static_pos_quality_costs;
    std::vector<double> pass_friendly_capability_costs;
    std::vector<double> pass_enemy_risk_costs;
    std::vector<double> pass_shoot_score_costs;

    // We loop column wise (int the same order as how zones are defined)
    for (int i = 0; i < NUM_COLS; i++)
    {
        // x coordinate of the centre of the column
        double x = width * i + width / 2 - world.field().xLength() / 2;

        for (int j = 0; j < NUM_ROWS; j++)
        {
            double y  = height * j + height / 2 - world.field().yLength() / 2;
            auto pass = Pass(world.ball().position(), Point(x, y),
                             passing_config_.max_pass_speed_m_per_s());

            // getStaticPositionQuality
            static_pos_quality_costs.push_back(getStaticPositionQuality(
                world.field(), pass.receiverPoint(), passing_config_));

            // ratePassFriendlyCapability
            pass_friendly_capability_costs.push_back(
                ratePassFriendlyCapability(world.friendlyTeam(), pass, passing_config_));

            // ratePassEnemyRisk
            pass_enemy_risk_costs.push_back(ratePassEnemyRisk(
                world.enemyTeam(), pass,
                Duration::fromSeconds(passing_config_.enemy_reaction_time()),
                passing_config_.enemy_proximity_importance()));

            // ratePassShootScore
            pass_shoot_score_costs.push_back(ratePassShootScore(
                world.field(), world.enemyTeam(), pass, passing_config_));
        }
    }

    func_to_costs["getStaticPositionQuality"]   = static_pos_quality_costs;
    func_to_costs["ratePassFriendlyCapability"] = pass_friendly_capability_costs;
    func_to_costs["ratePassEnemyRisk"]          = pass_enemy_risk_costs;
    func_to_costs["ratePassShootScore"]         = pass_shoot_score_costs;

    LOG(VISUALIZE) << *createCostVisualization(func_to_costs, NUM_ROWS, NUM_COLS);
}

template <class ZoneEnum>
ZonePassMap<ZoneEnum> PassGenerator<ZoneEnum>::optimizePasses(
    const World& world, const ZonePassMap<ZoneEnum>& generated_passes)
{
    // Run gradient descent to optimize the passes to for the requested number
    // of iterations
    ZonePassMap<ZoneEnum> optimized_passes;

    for (ZoneEnum zone_id : pitch_division_->getAllZoneIds())
    {
        // The objective function we minimize in gradient descent to improve each pass
        // that we're optimizing
        const auto objective_function =
            [this, &world,
             zone_id](const std::array<double, NUM_PARAMS_TO_OPTIMIZE>& pass_array) {
                return ratePass(world,
                                Pass::fromPassArray(world.ball().position(), pass_array),
                                pitch_division_->getZone(zone_id), passing_config_);
            };

        auto pass_array = optimizer_.maximize(
            objective_function, generated_passes.at(zone_id).pass.toPassArray(),
            passing_config_.number_of_gradient_descent_steps_per_iter());

        auto new_pass = Pass::fromPassArray(world.ball().position(), pass_array);
        auto score =
            ratePass(world, new_pass, pitch_division_->getZone(zone_id), passing_config_);

        optimized_passes.emplace(zone_id, PassWithRating{new_pass, score});
    }

    return optimized_passes;
}

template <class ZoneEnum>
void PassGenerator<ZoneEnum>::updatePasses(const World& world,
                                           const ZonePassMap<ZoneEnum>& optimized_passes)
{
    for (ZoneEnum zone_id : pitch_division_->getAllZoneIds())
    {
        // update the passer point of the current best pass
        current_best_passes_.at(zone_id).pass = Pass::fromPassArray(
            world.ball().position(), current_best_passes_.at(zone_id).pass.toPassArray());

        if (ratePass(world, current_best_passes_.at(zone_id).pass,
                     pitch_division_->getZone(zone_id),
                     passing_config_) < optimized_passes.at(zone_id).rating)
        {
            current_best_passes_.at(zone_id) = optimized_passes.at(zone_id);
        }
    }
}

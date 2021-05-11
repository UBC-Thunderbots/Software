#include <algorithm>
#include <chrono>
#include <numeric>

#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass_evaluation.h"
#include "software/ai/passing/pass_generator.h"

template <class ZoneEnum>
PassGenerator<ZoneEnum>::PassGenerator(
    std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division,
    std::shared_ptr<const PassingConfig> passing_config)
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
    auto generated_passes = samplePasses(world);
    if (current_best_passes_.empty())
    {
        current_best_passes_ = generated_passes;
    }
    auto optimized_passes = optimizePasses(world, generated_passes);

    updatePasses(world, optimized_passes);

    return PassEvaluation<ZoneEnum>(pitch_division_, current_best_passes_,
                                    passing_config_, world.getMostRecentTimestamp());
}

template <class ZoneEnum>
ZonePassMap<ZoneEnum> PassGenerator<ZoneEnum>::samplePasses(const World& world)
{
    std::uniform_real_distribution speed_distribution(
        passing_config_->getMinPassSpeedMPerS()->value(),
        passing_config_->getMaxPassSpeedMPerS()->value());

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
            passing_config_->getNumberOfGradientDescentStepsPerIter()->value());

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

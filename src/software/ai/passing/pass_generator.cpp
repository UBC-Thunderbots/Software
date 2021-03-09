
#include "software/ai/passing/pass_generator.h"

#include <algorithm>
#include <chrono>
#include <numeric>

#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass_evaluation.h"

template <class ZoneEnum>
PassGenerator<ZoneEnum>::PassGenerator(
    std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division)
    : optimizer_(optimizer_param_weights),
      pitch_division_(pitch_division),
      random_num_gen_(PASS_GENERATOR_SEED)
{
}

template <class ZoneEnum>
PassEvaluation<ZoneEnum> PassGenerator<ZoneEnum>::generatePassEvaluation(
    const World& world)
{
    auto generated_passes = samplePasses(world);
    if (passes_.empty())
    {
        passes_ = generated_passes;
    }
    auto optimized_passes = optimizePasses(world, generated_passes);
    updatePasses(world, optimized_passes);

    return PassEvaluation<ZoneEnum>(pitch_division_, passes_,
                                    world.getMostRecentTimestamp());
}

template <class ZoneEnum>
ZonePassMap<ZoneEnum> PassGenerator<ZoneEnum>::samplePasses(const World& world)
{
    std::uniform_real_distribution speed_distribution(DynamicParameters->getAiConfig()
                                                          ->getPassingConfig()
                                                          ->getMinPassSpeedMPerS()
                                                          ->value(),
                                                      DynamicParameters->getAiConfig()
                                                          ->getPassingConfig()
                                                          ->getMaxPassSpeedMPerS()
                                                          ->value());
    ZonePassMap<ZoneEnum> passes;

    // Randomly sample a pass in each zone
    for (ZoneEnum zone_id : pitch_division_->getAllZoneIds())
    {
        auto zone = pitch_division_->getZone(zone_id);

        std::uniform_real_distribution x_distribution(zone.xMin(), zone.xMax());
        std::uniform_real_distribution y_distribution(zone.yMin(), zone.yMax());

        auto pass =
            Pass(Point(x_distribution(random_num_gen_), y_distribution(random_num_gen_)),
                 speed_distribution(random_num_gen_));

        passes.emplace(zone_id, PassWithRating{
            pass, ratePass(world, pass, pitch_division_->getZone(zone_id))});
    }

    return passes;
}

template <class ZoneEnum>
ZonePassMap<ZoneEnum> PassGenerator<ZoneEnum>::optimizePasses(
    const World& world, const ZonePassMap<ZoneEnum>& generated_passes)
{
    // Run gradient descent to optimize the passes to for the requested number
    // of iterations
    // NOTE: Parallelizing this `for` loop would probably be safe and potentially more
    //       performant
    ZonePassMap<ZoneEnum> optimized_passes;

    for (ZoneEnum zone_id : pitch_division_->getAllZoneIds())
    {
        // The objective function we minimize in gradient descent to improve each pass
        // that we're optimizing
        const auto objective_function =
            [this, &world,
             zone_id](const std::array<double, NUM_PARAMS_TO_OPTIMIZE>& pass_array) {
                return ratePass(world, Pass::fromPassArray(pass_array),
                                pitch_division_->getZone(zone_id));
            };

        auto pass_array = optimizer_.maximize(
            objective_function, generated_passes.at(zone_id).pass.toPassArray(),
            DynamicParameters->getAiConfig()
                ->getPassingConfig()
                ->getNumberOfGradientDescentStepsPerIter()
                ->value());
        try
        {
            auto new_pass = Pass::fromPassArray(pass_array);
            auto score    = ratePass(world, new_pass, pitch_division_->getZone(zone_id));

            optimized_passes.emplace(zone_id, PassWithRating{new_pass, score});
        }
        catch (std::invalid_argument& e)
        {
            // Sometimes the gradient descent algorithm could return an invalid pass
            // (i.e a pass w/ a negative speed). We just keep the initial pass in that
            // case.
            optimized_passes.emplace(zone_id, generated_passes.at(zone_id));
        }
    }

    return optimized_passes;
}

template <class ZoneEnum>
void PassGenerator<ZoneEnum>::updatePasses(const World& world,
                                           const ZonePassMap<ZoneEnum>& optimized_passes)
{
    for (ZoneEnum zone_id : pitch_division_->getAllZoneIds())
    {
        if (ratePass(world, passes_.at(zone_id).pass, pitch_division_->getZone(zone_id)) <
            optimized_passes.at(zone_id).rating)
        {
            passes_.at(zone_id) = optimized_passes.at(zone_id);
        }
    }
}

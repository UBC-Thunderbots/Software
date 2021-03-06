#include "software/ai/passing/pass_generator.h"

#include <algorithm>
#include <numeric>

#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass_evaluation.h"
#include "software/ai/passing/pass_generator.h"

PassGenerator::PassGenerator(std::shared_ptr<const FieldPitchDivision>& pitch_division)
    : optimizer_(optimizer_param_weights),
      pitch_division_(pitch_division),
      random_num_gen_(PASS_GENERATOR_SEED)
{
}

PassEvaluation PassGenerator::generatePassEvaluation(const World& world)
{
    auto generated_passes = samplePasses(world);
    if (passes_.empty())
    {
        passes_ = generated_passes;
    }
    auto optimized_passes = optimizePasses(world, generated_passes);
    updatePasses(world, optimized_passes);

    return PassEvaluation(pitch_division_, passes_, world.getMostRecentTimestamp());
}

std::vector<PassWithRating> PassGenerator::samplePasses(const World& world)
{
    std::uniform_real_distribution speed_distribution(DynamicParameters->getAiConfig()
                                                          ->getPassingConfig()
                                                          ->getMinPassSpeedMPerS()
                                                          ->value(),
                                                      DynamicParameters->getAiConfig()
                                                          ->getPassingConfig()
                                                          ->getMaxPassSpeedMPerS()
                                                          ->value());
    std::vector<PassWithRating> passes;

    // Randomly sample a pass in each zone
    for (unsigned zone_id = 1; zone_id <= pitch_division_->getTotalNumberOfZones();
         ++zone_id)
    {
        auto zone = pitch_division_->getZone(zone_id);

        std::uniform_real_distribution x_distribution(zone.xMin(), zone.xMax());
        std::uniform_real_distribution y_distribution(zone.yMin(), zone.yMax());

        auto pass =
            Pass(Point(x_distribution(random_num_gen_), y_distribution(random_num_gen_)),
                 speed_distribution(random_num_gen_));

        passes.emplace_back(PassWithRating{pass, ratePass(world, pass)});
    }

    return passes;
}

std::vector<PassWithRating> PassGenerator::optimizePasses(
    const World& world, const std::vector<PassWithRating>& generated_passes)
{
    // The objective function we minimize in gradient descent to improve each pass
    // that we're optimizing
    const auto objective_function =
        [this, &world](const std::array<double, NUM_PARAMS_TO_OPTIMIZE>& pass_array) {
            try
            {
                return ratePass(world, Pass::fromPassArray(pass_array));
            }
            catch (std::invalid_argument& e)
            {
                return 0.0;
            }
        };

    // Run gradient descent to optimize the passes to for the requested number
    // of iterations
    // NOTE: Parallelizing this `for` loop would probably be safe and potentially more
    //       performant
    std::vector<PassWithRating> optimized_passes;
    for (const PassWithRating& pass_with_rating : generated_passes)
    {
        auto pass_array =
            optimizer_.maximize(objective_function, pass_with_rating.pass.toPassArray(),
                                DynamicParameters->getAiConfig()
                                    ->getPassingConfig()
                                    ->getNumberOfGradientDescentStepsPerIter()
                                    ->value());
        try
        {
            auto new_pass = Pass::fromPassArray(pass_array);
            optimized_passes.emplace_back(
                PassWithRating{new_pass, ratePass(world, new_pass)});
        }
        catch (std::invalid_argument& e)
        {
            // Sometimes the gradient descent algorithm could return an invalid pass
            // (i.e a pass w/ a negative speed). We just keep the initial pass in that
            // case.
            optimized_passes.emplace_back(pass_with_rating);
        }
    }

    return optimized_passes;
}

void PassGenerator::updatePasses(const World& world,
                                 const std::vector<PassWithRating>& optimized_passes)
{
    for (unsigned zone_id = 1; zone_id <= pitch_division_->getTotalNumberOfZones();
         ++zone_id)
    {
        if (ratePass(world, passes_[zone_id - 1].pass) <
            optimized_passes[zone_id - 1].rating)
        {
            passes_[zone_id - 1] = optimized_passes[zone_id - 1];
        }
    }
}

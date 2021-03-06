#include "software/ai/passing/pass_generator.h"

#include <algorithm>
#include <numeric>

#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass_generator.h"

PassGenerator::PassGenerator(const FieldPitchDivision& pitch_division)
    : optimizer(optimizer_param_weights),
      pitch_division(pitch_division),
      random_num_gen(PASS_GENERATOR_SEED),
{
}

PassEvaluation PassGenerator::getPassEvaluation(const World& world)
{
    auto generated_passes             = generatePasses();
    auto optimized_passes_with_rating = optimizePasses(generated_passes);
}

std::vector<PassWithRating> PassGenerator::generatePasses()
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
    for (unsigned zone_id = 1; zone_id < pitch_division.getTotalNumberOfZones();
         ++zone_id)
    {
        auto zone = pitch_division.getZone(zone_id);

        std::uniform_real_distribution x_distribution(zone.xMin(), zone.xMax());
        std::uniform_real_distribution y_distribution(zone.yMin(), zone.yMax());

        auto pass =
            Pass(Point(x_distribution(random_num_gen), y_distribution(random_num_gen)),
                 speed_distribution(random_num_gen));

        passes.emplace_back(PassWithRating{pass, ratePass(pass)});
    }

    return passes;
}

std::vector<PassWithRating> PassGenerator::optimizePasses(
    const std::vector<Pass>& generated_passes)
{
    // The objective function we minimize in gradient descent to improve each pass
    // that we're optimizing
    const auto objective_function =
        [this](const std::array<double, NUM_PARAMS_TO_OPTIMIZE>& pass_array) {
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
    std::vector<Pass> optimized_passes;
    for (Pass& pass : generated_passes)
    {
        auto pass_array =
            optimizer.maximize(objective_function, pass.toPassArray(),
                               DynamicParameters->getAiConfig()
                                   ->getPassingConfig()
                                   ->getNumberOfGradientDescentStepsPerIter()
                                   ->value());
        try
        {
            auto new_pass = Pass::fromPassArray(pass_array);
            optimized_passes.emplace_back(PassWithRating{new_pass, ratePass(new_pass)});
        }
        catch (std::invalid_argument& e)
        {
            // Sometimes the gradient descent algorithm could return an invalid pass
            // (i.e a pass w/ a negative speed). We just keep the initial pass in that
            // case.
            optimized_passes.emplace_back(PassWithRating{pass, ratePass(pass)});
        }
    }

    return optimized_passes;
}

#include "software/ai/passing/pass_generator.h"

#include <algorithm>
#include <numeric>

#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass_generator.h"

PassGenerator::PassGenerator()
    : optimizer(optimizer_param_weights),
      // We initialize the random number generator with a specific value to
      // allow generated passes to be deterministic. The value used here has
      // no special meaning.
      random_num_gen(13),
{
    // Generate the initial set of passes
    passes_to_optimize = generatePasses(getNumPassesToOptimize());
}

PassWithRating PassGenerator::getBestPassSoFar()
{
    optimizePasses();
    pruneAndReplacePasses();
    saveBestPass();
}

void PassGenerator::optimizePasses()
{
    // The objective function we minimize in gradient descent to improve each pass
    // that we're optimizing
    const auto objective_function =
        [this](const std::array<double, NUM_PARAMS_TO_OPTIMIZE>& pass_array) {
            try
            {
                Pass pass = convertArrayToPass(pass_array);
                return ratePass(world, pass);
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
    std::vector<Pass> updated_passes;
    for (Pass& pass : passes_to_optimize)
    {
        auto pass_array =
            optimizer.maximize(objective_function, convertPassToArray(pass),
                               DynamicParameters->getAiConfig()
                                   ->getPassingConfig()
                                   ->getNumberOfGradientDescentStepsPerIter()
                                   ->value());
        try
        {
            updated_passes.emplace_back(convertArrayToPass(pass_array));
        }
        catch (std::invalid_argument& e)
        {
            // Sometimes the gradient descent algorithm could return an invalid pass, if
            // so, we can just ignore it and carry on
        }
    }
    passes_to_optimize = updated_passes;
}

double PassGenerator::ratePass(const Pass& pass)
{
    double rating = 0;
    try
    {
        rating = ::ratePass(world, pass, target_region, passer_robot_id, pass_type);
    }
    catch (std::invalid_argument& e)
    {
        // If the pass is invalid, just rate it as poorly as possible
        rating = 0;
    }

    return rating;
}

std::vector<Pass> PassGenerator::generatePasses(unsigned long num_passes_to_gen)
{
    std::uniform_real_distribution x_distribution(-world.field().xLength() / 2,
                                                  world.field().xLength() / 2);
    std::uniform_real_distribution y_distribution(-world.field().yLength() / 2,
                                                  world.field().yLength() / 2);

    double curr_time             = world.getMostRecentTimestamp().toSeconds();
    double min_start_time_offset = DynamicParameters->getAiConfig()
                                       ->getPassingConfig()
                                       ->getMinTimeOffsetForPassSeconds()
                                       ->value();
    double max_start_time_offset = DynamicParameters->getAiConfig()
                                       ->getPassingConfig()
                                       ->getMaxTimeOffsetForPassSeconds()
                                       ->value();
    std::uniform_real_distribution start_time_distribution(
        curr_time + min_start_time_offset, curr_time + max_start_time_offset);
    std::uniform_real_distribution speed_distribution(DynamicParameters->getAiConfig()
                                                          ->getPassingConfig()
                                                          ->getMinPassSpeedMPerS()
                                                          ->value(),
                                                      DynamicParameters->getAiConfig()
                                                          ->getPassingConfig()
                                                          ->getMaxPassSpeedMPerS()
                                                          ->value());

    std::vector<Pass> passes;
    for (unsigned i = 0; i < num_passes_to_gen; i++)
    {
        Point receiver_point(x_distribution(random_num_gen),
                             y_distribution(random_num_gen));
        Timestamp start_time =
            Timestamp::fromSeconds(start_time_distribution(random_num_gen));
        double pass_speed = speed_distribution(random_num_gen);

        Pass p(passer_point, receiver_point, pass_speed, start_time);
        passes.emplace_back(p);
    }

    return passes;
}

bool PassGenerator::comparePassQuality(const Pass& pass1, const Pass& pass2)
{
    return ratePass(pass1) > ratePass(pass2);
}

std::array<double, PassGenerator::NUM_PARAMS_TO_OPTIMIZE>
PassGenerator::convertPassToArray(const Pass& pass)
{
    return {pass.receiverPoint().x(), pass.receiverPoint().y(), pass.speed(),
            pass.startTime().toSeconds()};
}

Pass PassGenerator::convertArrayToPass(
    const std::array<double, PassGenerator::NUM_PARAMS_TO_OPTIMIZE>& array)
{
    // Clamp the time to be >= 0, otherwise the TimeStamp will throw an exception
    double time_offset_seconds = std::max(0.0, array.at(3));

    return Pass(passer_point, Point(array.at(0), array.at(1)), array.at(2),
                Timestamp::fromSeconds(time_offset_seconds));
}

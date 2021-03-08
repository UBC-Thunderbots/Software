
#include "software/ai/passing/pass_generator.h"

#include <algorithm>
#include <numeric>
#include <chrono>

#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass_evaluation.h"

PassGenerator::PassGenerator(std::shared_ptr<const FieldPitchDivision> pitch_division)
    : optimizer_(optimizer_param_weights),
      pitch_division_(pitch_division),
      random_num_gen_(PASS_GENERATOR_SEED)
{
}

PassEvaluation PassGenerator::generatePassEvaluation(const World& world)
{
    auto start = std::chrono::system_clock::now();
    auto generated_passes = samplePasses(world);
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout <<"samplePasses: "<< elapsed.count() << '\n';
    if (passes_.empty())
    {
        passes_ = generated_passes;
    }
    start = std::chrono::system_clock::now();
    auto optimized_passes = optimizePasses(world, generated_passes);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "optimizePasses: " << elapsed.count() << '\n';

    start = std::chrono::system_clock::now();
    updatePasses(world, optimized_passes);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "updatePasses: " << elapsed.count() << '\n';

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

        passes.emplace_back(PassWithRating{
            pass, ratePass(world, pass, pitch_division_->getZone(zone_id))});
    }

    return passes;
}

template <class E>
std::vector<PassWithRating> PassGenerator::optimizePasses(
    const World& world, const std::vector<PassWithRating>& generated_passes)
{
    unsigned zone = 1;

    // The objective function we minimize in gradient descent to improve each pass
    // that we're optimizing
    const auto objective_function =
        [this, &world,
         zone](const std::array<double, NUM_PARAMS_TO_OPTIMIZE>& pass_array) {
            return ratePass(world, Pass::fromPassArray(pass_array),
                            pitch_division_->getZone(zone));
        };

    // Run gradient descent to optimize the passes to for the requested number
    // of iterations
    // NOTE: Parallelizing this `for` loop would probably be safe and potentially more
    //       performant
    std::vector<PassWithRating> optimized_passes;
    for (zone = 1; zone <= pitch_division_->getTotalNumberOfZones(); ++zone)
    {
        auto start            = std::chrono::system_clock::now();
        auto pass_array = optimizer_.maximize(
            objective_function, generated_passes[zone - 1].pass.toPassArray(), 10);
        auto end              = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout <<"optimizer: "<< elapsed.count() << '\n';
        try
        {
            auto new_pass = Pass::fromPassArray(pass_array);
            auto old_score = ratePass(world, generated_passes[zone - 1].pass, pitch_division_->getZone(zone));
            auto score = ratePass(world, new_pass, pitch_division_->getZone(zone));

            std::cerr<<score - old_score<<std::endl;
            optimized_passes.emplace_back(PassWithRating{new_pass, score});
        }
        catch (std::invalid_argument& e)
        {
            // Sometimes the gradient descent algorithm could return an invalid pass
            // (i.e a pass w/ a negative speed). We just keep the initial pass in that
            // case.
            optimized_passes.emplace_back(generated_passes[zone - 1]);
        }
    }

    return optimized_passes;
}
 
template <class E>
void PassGenerator::updatePasses(const World& world,
                                 const std::vector<PassWithRating>& optimized_passes)
{
    for (E zone_id = 1; zone_id <= pitch_division_->getTotalNumberOfZones(); ++zone_id)
    {
        if (ratePass(world, passes_[zone_id].pass, pitch_division_->getZone(zone_id)) <
            optimized_passes[zone_id].rating)
        {
            passes_[zone_id] = optimized_passes[zone_id];
        }
    }
}

#pragma once

#include <mutex>
#include <random>
#include <thread>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass.h"
#include "software/ai/passing/pass_evaluation.h"
#include "software/ai/passing/pass_with_rating.h"
#include "software/optimization/gradient_descent_optimizer.h"
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
        std::shared_ptr<const PassingConfig> passing_config);

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
    std::shared_ptr<const PassingConfig> passing_config_;

    // A random number generator for use across the class
    std::mt19937 random_num_gen_;
};

#include "software/ai/passing/pass_generator.hpp"

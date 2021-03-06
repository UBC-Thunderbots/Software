#pragma once

#include <mutex>
#include <random>
#include <thread>

#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass.h"
#include "software/ai/passing/pass_with_rating.h"
#include "software/ai/passing/pass_evaluation.h"
#include "software/optimization/gradient_descent_optimizer.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

const int PASS_GENERATOR_SEED = 13;

/**
 * This class is responsible for generating passes for us to perform
 */
class PassGenerator
{
   public:
    PassGenerator& operator=(const PassGenerator&) = delete;
    PassGenerator(const PassGenerator&)            = delete;
    PassGenerator(const FieldPitchDivision& pitch_division) = default;

    /**
     * Creates a PassEvaluation given a world and a field pitch division.
     *
     * NOTE: It is important that this function is able to run in 10ms.
     *
     * This is because a pass is evaluated on the provided world and if the
     * pass generator only completes an iteration of pass updates once every 5 seconds,
     * very soon when optimizing the pass), and so the passes will likely be invalid by
     * the time another iteration starts. Because of this, it is extremely important that
     * the pass generator runs fast enough. Debug builds running on slightly slower
     * computers could be unable to converge. It is recommended that all testing of things
     * involving the PassGenerator be done with executables built in "Release" in order to
     * maximize performance ("Release" can be 2-10x faster then "Debug").
     *
     * @param world The world to compute the pass evaluation on
     *
     * @return The best currently known pass and the rating of that pass (in [0-1])
     */
    PassEvaluation getPassEvaluation(const World& world);


   private:
    // The number of parameters (representing a pass) that we optimize
    // (pass_start_x, pass_start_y, pass_speed)
    static const int NUM_PARAMS_TO_OPTIMIZE = 3;

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
     * Updates, Optimizes, And Prunes the Passes
     */
    std::vector<Pass> generatePasses();

    /**
     * Optimizes all current passes
     */
    std::vector<PassWithRating> optimizePasses(
                const std::vector<PassWithRating>& generated_passes);

    /**
     * Prunes un-promising passes and replaces them with newly generated ones
     */
    void prunePasses();

    // This world is what is used in the optimization loop
    World world;

    // All the passes that we are currently trying to optimize in gradient descent
    std::vector<Pass> passes_to_optimize;

    // The optimizer we're using to find passes
    GradientDescentOptimizer<NUM_PARAMS_TO_OPTIMIZE> optimizer;

    // A random number generator for use across the class
    std::mt19937 random_num_gen;

    // Pitch division
    FieldPitchDivision pitch_division;
};

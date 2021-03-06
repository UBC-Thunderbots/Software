#pragma once

#include <mutex>
#include <random>
#include <thread>

#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass.h"
#include "software/ai/passing/pass_evaluation.h"
#include "software/ai/passing/pass_with_rating.h"
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
    /**
     * Creates a new PassGenerator with the given pitch_division.
     *
     * The PassGenerator will use this pitch division to guide initial random samples
     * in each zone after the pitch has been divided.
     *
     * @param pitch_division The pitch division to use when looking for passes
     */
    explicit PassGenerator(std::shared_ptr<const FieldPitchDivision>& pitch_division);

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
    PassEvaluation generatePassEvaluation(const World& world);


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
     * Randomly samples a recieve point across every zone and assigns a random
     * speed to each pass.
     *
     * @returns a vector of sampled passes
     */
    std::vector<Pass> samplePasses();

    /**
     * Given a vector of passes, runs a gradient descent optimizer to find
     * better passes
     *
     * @param The world
     * @param The passes to be optimized
     * @returns The optimized passes
     */
    std::vector<PassWithRating> optimizePasses(const World& world,
                                               const std::vector<Pass>& initial_passes);

    /**
     * Re-evaluates ratePass on the "previous ticks" passes and keeps the higher pass
     * w/ the higher score in passes_;
     *
     * @param The world
     * @param optimized_passes The optimized_passes to update our internal passes with.
     */
    void updatePasses(const World& world,
                      const std::vector<PassWithRating>& optimized_passes);

    // All the passes that we are currently trying to optimize in gradient descent
    std::vector<PassWithRating> passes_;

    // The optimizer we're using to find passes
    GradientDescentOptimizer<NUM_PARAMS_TO_OPTIMIZE> optimizer_;

    // Pitch division
    std::shared_ptr<const FieldPitchDivision> pitch_division_;

    // A random number generator for use across the class
    std::mt19937 random_num_gen_;
};

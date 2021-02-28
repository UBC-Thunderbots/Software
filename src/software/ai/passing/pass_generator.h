#pragma once

#include <mutex>
#include <random>
#include <thread>

#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass.h"
#include "software/ai/passing/pass_with_rating.h"
#include "software/optimization/gradient_descent_optimizer.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

/**
 * This class is responsible for generating passes for us to perform
 *
 * == General Description ==
 * On optimizing/pruning/re-generating passes. As such, any
 * modifications to data in this class through a public interface *must* be managed
 * by mutexes. Generally in functions that touch data we use "std::lock_guard" to
 * take ownership of the mutex protecting that data for the duration of the function.
*
 * == Performance Considerations ==
 * It is important that the pass generator is able to "keep up" with the current
 * time. This is because a pass is defined in part by it's start time. If the
 * pass generator only completes an iteration of pass updates once every 5 seconds,
 * then the start times for the passes will be in the past (if we choose to pass
 * very soon when optimizing the pass), and so the passes will likely be invalid by
 * the time another iteration starts. Because of this, it is extremely important that
 * the pass generator runs fast enough. Debug builds running on slightly slower
 * computers could be unable to converge. It is recommended that all testing of things
 * involving the PassGenerator be done with executables built in "Release" in order to
 * maximize performance ("Release" can be 2-10x faster then "Debug").
 */
class PassGenerator
{
   public:
    // Delete the default constructor, we want to force users to choose what
    // pass quality they deem reasonable
    PassGenerator(FieldPitchDivsion pitch_division) = delete;

    /**
     * Gets the best pass that the PassGenerator has generated so far. The pass generator
     * may do some work between calls, and getBestPassSoFar will eventually return better
     * passes over multiple calls.  The best strategy for getting good passes is to call
     * getBestPassSoFar multiple times, until the pass is rated high enough.
     *
     * @return The best currently known pass and the rating of that pass (in [0-1])
     */
    PassEvaluation getPassEvaluation(const World& world);


   private:
    // The number of parameters (representing a pass) that we optimize
    // (pass_start_x, pass_start_y, pass_speed, pass_start_time)
    static const int NUM_PARAMS_TO_OPTIMIZE = 4;

    // Weights used to normalize the parameters that we pass to GradientDescent
    // (see the GradientDescent documentation for details)
    // These weights are *very* roughly the step that gradient descent will take
    // in each respective dimension for a single iteration. They are tuned to
    // ensure passes converge as fast as possible, but are also as stable as
    // possible
    static constexpr double PASS_SPACE_WEIGHT                          = 0.1;
    static constexpr double PASS_TIME_WEIGHT                           = 0.1;
    static constexpr double PASS_SPEED_WEIGHT                          = 0.01;
    std::array<double, NUM_PARAMS_TO_OPTIMIZE> optimizer_param_weights = {
        PASS_SPACE_WEIGHT, PASS_SPACE_WEIGHT, PASS_TIME_WEIGHT, PASS_SPEED_WEIGHT};

    /**
     * Updates, Optimizes, And Prunes the Passes
     */
    void updateAndOptimizeAndPrunePasses();

    /**
     * Optimizes all current passes
     */
    void optimizePasses();

    /**
     * Prunes un-promising passes and replaces them with newly generated ones
     */
    void pruneAndReplacePasses();

    /**
     * Saves the best currently known pass
     */
    void saveBestPass();

    /**
     * Get the number of passes to keep after pruning
     *
     * @return the number of passes to keep after pruning
     */
    unsigned int getNumPassesToKeepAfterPruning();

    /**
     * Get the number of passes to optimize
     *
     * @return the number of passes to optimize
     */
    unsigned int getNumPassesToOptimize();

    /**
     * Convert the given pass to an array
     *
     * @param pass The pass to convert
     *
     * @return An array containing the parts of the pass we want to optimize, in the
     *         form: {receiver_point.x, receiver_point.y, pass_speed_m_per_s
     *                pass_start_time}
     */
    std::array<double, NUM_PARAMS_TO_OPTIMIZE> convertPassToArray(const Pass& pass);

    /**
     * Convert a given array to a Pass
     *
     * @param array The array to convert to a pass, in the form:
     *              {receiver_point.x, receiver_point.y, pass_speed_m_per_s,
     *              pass_start_time}
     *
     * @return The pass represented by the given array, with the passer point being
     *         the current `passer_point` we're optimizing for
     */
    Pass convertArrayToPass(const std::array<double, NUM_PARAMS_TO_OPTIMIZE>& array);

    /**
     * Calculate the quality of a given pass
     * @param pass The pass to rate
     * @return A value in [0,1] representing the quality of the pass with 1 being the
     *         best pass and 0 being the worst pass
     */
    double ratePass(const Pass& pass);

    /**
     * Updates the passer point of all passes that we're currently optimizing
     *
     * @param new_passer_point The new passer point
     */
    void updatePasserPointOfAllPasses(const Point& new_passer_point);

    /**
     * Compares the quality of the two given passes
     *
     * @param pass1
     * @param pass2
     *
     * @return pass1.quality < pass2.quality
     */
    bool comparePassQuality(const Pass& pass1, const Pass& pass2);

    /**
     * Check if the two given passes are equal
     *
     * Equality here is defined in the context of this class, in that we use it as a
     * measure of whether or not to merge two passes
     *
     * @param pass1
     * @param pass2
     *
     * @return True if the two passes are similar enough to be equal, false otherwise
     */
    bool passesEqual(Pass pass1, Pass pass2);

    /**
     * Generate a given number of passes
     *
     * This function is used to generate the initial passes that are then optimized
     * via gradient descent.
     *
     * @param num_passes_to_gen  The number of passes to generate
     *
     * @return A vector containing the requested number of passes
     */
    std::vector<Pass> generatePasses(unsigned long num_passes_to_gen);

    // This world is what is used in the optimization loop
    World world;

    // All the passes that we are currently trying to optimize in gradient descent
    std::vector<Pass> passes_to_optimize;

    // The optimizer we're using to find passes
    GradientDescentOptimizer<NUM_PARAMS_TO_OPTIMIZE> optimizer;

    // A random number generator for use across the class
    std::mt19937 random_num_gen;
};

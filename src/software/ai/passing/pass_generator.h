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
 * It is built such that when it is constructed, a thread is immediately started in
 * the background continuously optimizing/pruning/re-generating passes. As such, any
 * modifications to data in this class through a public interface *must* be managed
 * by mutexes. Generally in functions that touch data we use "std::lock_guard" to
 * take ownership of the mutex protecting that data for the duration of the function.
 *
 * == Making Changes/Additions ==
 * Whenever you change/add a function, you need to ask: "what data is this _directly_
 * touching?". If the function is touching anything internal to the class, you should
 * be getting a lock on the mutex for that data member for the duration of the
 * function (see below for examples). Note the *directly* bit! If you are
 * changing/adding function "A", and you have it call function "B", if B touches
 * data, then B should be responsible for getting a lock on the data. If you acquire
 * a lock in A, then call B, which also requires a lock, then the threads will
 * deadlock and everything will grind to a halt.
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
    PassGenerator() = delete;

    // Delete the copy and assignment operators because this class really shouldn't
    // need them and we don't want to risk doing anything nasty with the internal
    // threading this class uses
    PassGenerator& operator=(const PassGenerator&) = delete;
    PassGenerator(const PassGenerator&)            = delete;

    /**
     * Create a PassGenerator with given parameters
     *
     * @param world The world we're passing int
     * @param passer_point The point we're passing from
     * @param pass_type The type of pass we would like to perform.
     *                  NOTE: this will _try_ to generate a pass of the type given,
     *                  but it is not guaranteed, and can change during pass
     *                  execution because of Passer/Receiver decisions
     * @param passing_config The dynamic parameter config to use to tune the PassGenerator
     * @param run_deterministically If true, this disables all threading so that
     *                              the same sequence of calls to this function always
     *                              returns the same values, no matter how much time
     *                              passes between calls
     */
    explicit PassGenerator(const World& world, const Point& passer_point,
                           const PassType& pass_type,
                           std::shared_ptr<const PassingConfig> passing_config,
                           const bool run_deterministically = false);

    /**
     * Updates the world
     *
     * @param world
     */
    void setWorld(World world);

    /**
     * Updates the point that we are passing from
     *
     * @param passer_point the point we are passing from
     */
    void setPasserPoint(Point passer_point);

    /**
     * Set the id of the robot performing the pass.
     *
     * This id will be used so we ignore the passer when determining where to
     * pass to. We assume this robot is on the friendly team.
     *
     * @param robot_id The id of the robot performing the pass
     */
    void setPasserRobotId(unsigned int robot_id);

    /**
     * Set the target region that we would like to pass to
     *
     * @param area An optional that may contain the area to pass to. If the
     *             optional is empty (ie. `std::nullopt`) then this indicates
     *             that there is no target region
     */
    void setTargetRegion(std::optional<Rectangle> area);

    /**
     * Gets the best pass that the PassGenerator has generated so far. The pass generator
     * may do some work between calls, and getBestPassSoFar will eventually return better
     * passes over multiple calls.  The best strategy for getting good passes is to call
     * getBestPassSoFar multiple times, until the pass is rated high enough.
     *
     * @return The best currently known pass and the rating of that pass (in [0-1])
     */
    PassWithRating getBestPassSoFar();

    /**
     * Destructs this PassGenerator
     */
    ~PassGenerator();


   private:
    // The number of parameters (representing a pass) that we optimize
    // (pass_start_x, pass_start_y, pass_speed, pass_start_time)
    static const int NUM_PARAMS_TO_OPTIMIZE = 4;

    // The number of iterations to run on each call to `getBestPassSoFar()`
    // **if** running deterministically.
    // This value was determined to work for unit tests, it will likely need to be
    // configurable in the future.
    static const size_t NUM_ITERS_PER_DETERMINISTIC_CALL = 10;

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
     * Continuously optimizes, prunes, and re-generates passes based on known info
     *
     * This will only return when the in_destructor flag is set to true
     */
    void continuouslyGeneratePasses();

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
     * Draws all the passes we are currently optimizing and the gradient of pass
     * receive position quality over the field
     */
    void visualizePassesAndPassQualityGradient();

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

    // Whether or not this generator is running deterministically (ie. threading
    // disabled so that the same sequence of calls to this function always returns
    // the same values, regardless of the time between calls)
    bool running_deterministically;

    // The thread running the pass optimization/pruning/re-generation in the
    // background. This thread will run for the entire lifetime of the class
    std::thread pass_generation_thread;

    // The mutex for the updated world
    std::mutex updated_world_mutex;

    // This world is the most recently updated one. We use this variable to "buffer"
    // the most recently updated world so that the world stays the same for the
    // entirety of each optimization loop, which makes things easier to reason about
    World updated_world;

    // The mutex for the world
    std::mutex world_mutex;

    // This world is what is used in the optimization loop
    World world;

    // The mutex for the passer robot ID
    std::mutex passer_robot_id_mutex;

    // The id of the robot that is performing the pass. We want to ignore this robot
    std::optional<unsigned int> passer_robot_id;

    // All the passes that we are currently trying to optimize in gradient descent
    std::vector<Pass> passes_to_optimize;

    // The optimizer we're using to find passes
    GradientDescentOptimizer<NUM_PARAMS_TO_OPTIMIZE> optimizer;

    // The mutex for the passer_point
    std::mutex passer_point_mutex;

    // The point we are passing from
    Point passer_point;

    // The mutex for the passer_point
    std::mutex best_known_pass_mutex;

    // The best pass we currently know about
    Pass best_known_pass;

    // The mutex for the target region
    std::mutex target_region_mutex;

    // The area that we want to pass to
    std::optional<Rectangle> target_region;

    // A random number generator for use across the class
    std::mt19937 random_num_gen;

    // What type of pass we're trying to generate
    PassType pass_type;

    // The mutex for the in_destructor flag
    std::mutex in_destructor_mutex;

    std::shared_ptr<const PassingConfig> passing_config;

    // This flag is used to indicate that we are in the destructor. We use this to
    // communicate with pass_generation_thread that it is
    // time to stop
    bool in_destructor;
};

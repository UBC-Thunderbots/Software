#pragma once

#include <thread>
#include <mutex>

#include "ai/passing/pass.h"
#include "ai/world/world.h"
#include "util/gradient_descent.h"
#include "util/parameter/dynamic_parameters.h"
#include "util/timestamp.h"

namespace AI::Passing {

    // TODO: detailed Javadoc comment for this class
    // TODO: explain the general idiom of: get in, lock what you need, exit (RAII?)
    // TODO: explain how to make changes to this class:
    //      - if you add data, consider if it could be accessed by two threads at once. If so, it needs a mutex
    //      - for each function added/changed, consider: what data does this access? If any of that data has a mutex, you must take a `lock_guard` for it
    //      (NOTE: this is only for data accessed _directly_ by the function, any data accessed by functions called by the function should be in _those_ functions
    class PassGenerator {

    public:
        // Delete the default constructor, we want to force users to choose what
        // pass quality they deem reasonable
        PassGenerator() = delete;

        // Delete the copy and assignment operators because this class really shouldn't
        // need them and we don't want to risk doing anything nasty with the internal
        // threading this class uses
        PassGenerator & operator=(const PassGenerator&) = delete;
        PassGenerator(const PassGenerator&) = delete;

        /**
         * Create a PassGenerator with given parameters
         *
         * @param min_reasonable_pass_quality A value in [0,1] representing the minimum
         *                                    quality for a pass to be considered
         *                                    "reasonable"
         */
        explicit PassGenerator(double min_reasonable_pass_quality);

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

        // TODO: Ability to set a target region

        /**
         * Gets the best pass we know of so far
         *
         * This only returns what we know so far. For example, if called directly after
         * the world state is updated, it is unlikely to return good results (if any).
         * Gradient descent must be allowed to run for some number of iterations before
         * this can be used to get a reasonable value.
         *
         * @return The best currently known pass, or `std::nullopt` if there is no
         *         reasonable pass
         */
        std::optional<Pass> getBestPassSoFar();

        /**
         * Destructs this PassGenerator
         */
        ~PassGenerator();


    private:
        // The number of parameters (representing a pass) that we optimize
        // (pass_start_x, pass_start_y, pass_speed, pass_start_time)
        static const int NUM_PARAMS_TO_OPTIMIZE = 4;

        // The number of passes to try to optimize at any given time
        // TODO: should this be a constant here? Maybe a dynamic parameter?
        static const unsigned int num_passes_to_optimize = 50;

        // The number of passes to keep after pruning
        // TODO: should this be a constant here? Maybe a dynamic parameter?
        static const unsigned int num_passes_to_keep_after_pruning = 10;

        // The number of steps of gradient descent to perform in each iteration
        // TODO: should this be a constant here? Maybe a dynamic parameter?
        static const unsigned int number_of_gradient_descent_steps_per_iter = 20;

        // Weights used to normalize the parameters that we pass to GradientDescent
        // (see the GradientDescent documentation for details)
        // TODO: should these be constants here? Maybe a dynamic parameter?
        static constexpr double PASS_SPACE_WEIGHT = 0.01;
        static constexpr double PASS_TIME_WEIGHT = 1;
        static constexpr double PASS_SPEED_WEIGHT = 1;
        std::array<double, NUM_PARAMS_TO_OPTIMIZE> optimizer_param_weights = {
                PASS_SPACE_WEIGHT,
                PASS_SPACE_WEIGHT,
                PASS_TIME_WEIGHT,
                PASS_SPEED_WEIGHT
        };

        /**
         * Continuously optimizes, prunes, and re-generates passes based on known info
         *
         * This will only return when the in_destructor flag is set
         */
        void continuouslyGeneratePasses();

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
        void saveBestPath();

        /**
         * Convert the given pass to an array
         *
         * @param pass The pass to convert
         *
         * @return An array containing the parts of the pass we want to optimize, in the
         *         form: {receiver_point.x, receiver_point.y, pass_speed_m_per_s
         *                pass_start_time}
         */
        static std::array<double, NUM_PARAMS_TO_OPTIMIZE> convertPassToArray(Pass pass);

        /**
         * Convert a given array to a Pass
         *
         * // TODO: better comment here?
         * Assumes that the passer point is the passer point held in this class
         *
         * @param array The array to convert to a pass, in the form:
         *              {receiver_point.x, receiver_point.y, pass_speed_m_per_s,
         *              pass_start_time}
         *
         * @return The pass represented by the given array
         */
        Pass convertArrayToPass(std::array<double, NUM_PARAMS_TO_OPTIMIZE> array);

        /**
         * Calculate the quality of a given pass
         * * @param pass The pass to rate
         * @return A value in [0,1] representing the quality of the pass
         */
        double ratePass(Pass pass);

        /**
         * Compares the quality of the two given passes
         *
         * @param pass1
         * @param pass2
         * @return pass1.quality < pass2.quality
         */
        bool comparePassQuality(const Pass& pass1, const Pass& pass2);

        /**
         * Generate a given number of passes
         *
         * This function is used to generate the initial passes that are then optimized via
         * gradient descent.
         *
         * @param num_passes_to_gen  The number of passes to generate
         *
         * @return A vector containing the requested number of passes
         */
        std::vector<Pass> generatePasses(unsigned long num_passes_to_gen);

        // This constant is used to prevent division by 0 in our implementation of Adam
        // (gradient descent)
        static constexpr double eps = 1e-8;

        // The minimum pass quality that we would consider a "reasonable" pass
        double min_reasonable_pass_quality;

        // The thread running the pass optimization/pruning/re-generation in the
        // background. This thread will run for the entire lifetime of the class
        std::thread pass_generation_thread;

        // The mutex for the in_destructor flag
        std::mutex in_destructor_mutex;

        // This flag is used to indicate that we are in the desctructor. We use this to
        // communicate with pass_generation_thread that it is
        // time to stop
        bool in_destructor;

        // The mutex for the world
        std::mutex world_mutex;

        // The most recent world we know about
        World world;

        // The mutex for the passer_point
        std::mutex passer_point_mutex;

        // The point we are passing from
        Point passer_point;

        // The mutex for the passer_point
        std::mutex best_known_pass_mutex;

        // The best pass we currently know about
        std::optional<Pass> best_known_pass;

        // All the passes that we are currently trying to optimize in gradient descent
        std::vector<Pass> passes_to_optimize;

        // The optimizer we're using to find passes
        Util::GradientDescentOptimizer<NUM_PARAMS_TO_OPTIMIZE> optimizer;

    };


}


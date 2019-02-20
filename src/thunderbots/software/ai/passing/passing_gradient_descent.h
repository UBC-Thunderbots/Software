#pragma once

#include <util/parameter/dynamic_parameters.h>
#include "ai/world/world.h"
#include "ai/passing/pass.h"
#include "util/timestamp.h"
#include "util/gradient_descent.h"

namespace AI::Passing {

    // TODO: detailed Javadoc comment for this class
    class PassGenerator {

    public:
        /**
         * Create a PassingGradientDescent
         */
        PassGenerator();

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

        // TODO: better comment here?
        /**
         * Runs a full iteration of pass optimization, pruning, and generation
         */
        void iterate();

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
        std::optional<Pass> getBestPass();

    private:

        // The number of parameters (representing a pass) that we optimize
        // (pass_start_x, pass_start_y, pass_speed, pass_start_time)
        static const int NUM_PARAMS_TO_OPTIMIZE = 4;

        /**
         * Convert the given pass to an array
         *
         * @param pass The pass to convert
         *
         * @return An array containing the parts of the pass we want to optimize
         */
        static std::array<double, NUM_PARAMS_TO_OPTIMIZE> convertPassToArray(Pass pass);

        /**
         * Convert a given array to a Pass
         *
         * // TODO: better comment here?
         * Assumes that the passer point is the passer point held in this class
         *
         * @param array The array to convert to a pass
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
        bool comparePassQuality(Pass pass1, Pass pass2);

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

        // The number of steps of gradient descent to perform in each iteration
        unsigned int number_of_gradient_descent_steps_per_iter;

        // The number of passes to use for gradient descent. This is the number of passes
        // that we will be trying to optimize at any given time
        unsigned int num_passes_to_optimize;

        // The minimum pass quality that we would consider a "reasonable" pass
        double min_reasonable_pass_quality;

        // The most recent world we know about
        World world;

        // The point we are passing from
        Point passer_point;

        // All the passes that we are currently trying to optimize in gradient descent
        std::vector<Pass> passes_to_optimize;

        // The optimizer we're using to find passes
        Util::GradientDescentOptimizer<NUM_PARAMS_TO_OPTIMIZE> optimizer;
    };


}


#pragma once

#include <random>

#include "pass_feature_collector.h"
#include "proto/parameters.pb.h"
#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass_with_rating.h"
#include "software/optimization/gradient_descent_optimizer.hpp"
#include "software/world/world.h"

/**
 * This class is responsible for generating passes using a random sampling method
 */
class PassGenerator
{
   public:
    /**
     * Constructs a new Sampling Pass Generator with the given config information
     *
     * @param passing_config The config to use when generating passes
     */
    explicit PassGenerator(const TbotsProto::PassingConfig& passing_config);

    /**
     * Generates the best pass based on the state of the world
     *
     * @param world The state of the world
     * @param robots_to_ignore A list of robot ids to ignore when generating passes
     *
     * @return The best pass that can be made and its rating
     */
    PassWithRating getBestPass(const World& world,
                               const std::vector<RobotId>& robots_to_ignore = {});

   private:
    /**
     * Randomly sample receiving points around friendly robots not included in the ignore
     * list
     *
     * @param world The current state of the world
     * @param robots_to_ignore A list of robot ids to ignore when generating passes
     *
     * @return a map of sampled points around friendly robots
     */
    std::map<RobotId, std::vector<Point>> sampleReceivingPositionsPerRobot(
        const World& world, const std::vector<RobotId>& robots_to_ignore);

    /**
     * Given a map of passes, runs a gradient descent optimizer to find
     * Update better passes.
     *
     * @param The world
     * @param The pass receiver position to be optimized mapped to robots
     * @returns Best optimized pass
     */
    PassWithRating optimizeReceivingPositions(
        const World& world,
        const std::map<RobotId, std::vector<Point>>& receiving_positions_map);

    // Weights used to normalize the parameters that we pass to GradientDescent
    // (see the GradientDescent documentation for details)
    // These weights are *very* roughly the step that gradient descent will take
    // in each respective dimension for a single iteration. They are tuned to
    // ensure passes converge as fast as possible, but are also as stable as
    // possible
    static constexpr double PASS_SPACE_WEIGHT                          = 0.1;
    std::array<double, NUM_PARAMS_TO_OPTIMIZE> optimizer_param_weights = {
        PASS_SPACE_WEIGHT, PASS_SPACE_WEIGHT};

    // The optimizer we're using to find passes
    GradientDescentOptimizer<NUM_PARAMS_TO_OPTIMIZE> optimizer_;

    std::map<RobotId, Point> previous_best_receiving_positions_;

    // the random seed used to initialize the random number generator
    static constexpr int RNG_SEED = 1010;

    // A random number generator for use across the class
    std::mt19937 random_num_gen_;

    // Passing configuration
    TbotsProto::PassingConfig passing_config_;

    // Pass Feature Collector
    PassFeatureCollector pass_feature_collector_;
    unsigned int num_passes_since_sample_ = 0;
    bool sample_pass_features_ = false;
};

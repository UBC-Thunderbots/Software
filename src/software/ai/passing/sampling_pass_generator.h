#pragma once

#include <random>

#include "proto/parameters.pb.h"
#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass_with_rating.h"
#include "software/world/world.h"

/**
 * This class is responsible for generating passes using a random sampling method
 */
class SamplingPassGenerator
{
   public:
    /**
     * Constructs a new Sampling Pass Generator with the given config information
     *
     * @param passing_config The config to use when generating passes
     */
    explicit SamplingPassGenerator(const TbotsProto::PassingConfig& passing_config);

    /**
     * Generates the best pass based on the state of the world
     *
     * @param world The state of the world
     * @param robots_to_ignore A list of robot ids to ignore when generating passes
     *
     * @return The best pass that can be made and its rating
     */
    PassWithRating getBestPass(const World& world, const std::vector<RobotId>& robots_to_ignore = {});

   private:
    /**
     * Randomly sample receiving points around friendly robots not included in the ignore list
     *
     * @param world The current state of the world
     * @param robots_to_ignore A list of robot ids to ignore when generating passes
     *
     * @return a list of sampled points around all friendly robots
     */
    std::vector<Point> sampleReceivingPositions(const World &world, const std::vector<RobotId>& robots_to_ignore);

    // the random seed used to initialize the random number generator
    static constexpr int RNG_SEED = 24;

    // A random number generator for use across the class
    std::mt19937 random_num_gen_;

    // Passing configuration
    TbotsProto::PassingConfig passing_config_;
};

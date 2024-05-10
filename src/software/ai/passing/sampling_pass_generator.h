#pragma once

#include <random>

#include "base_pass_generator.h"
#include "proto/parameters.pb.h"
#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass_with_rating.h"
#include "software/world/world.h"

// the random seed used to initialize the random number generator
static const int SEED = 14;

/**
 * This class is responsible for generating passes using a sampling method
 */
class SamplingPassGenerator : public BasePassGenerator
{
   public:
    /**
     * Constructs a new Sampling Pass Generator with the given config information
     *
     * @param passing_config The config to use when generating passes
     */
    SamplingPassGenerator(TbotsProto::PassingConfig passing_config);

    virtual PassWithRating getBestPass(const World& world);

   private:
    /**
     * Samples NUM_POINTS_TO_SAMPLE_PER_ROBOT points around all friendly robots
     * using a normal distribution, with mean being the position of the robot and
     * a standard deviation of SAMPLING_SD_METERS
     *
     * @param world The current state of the world
     *
     * @return a list of sampled points around all friendly robots
     */
    std::vector<Point> samplePasses(const World& world);

    // A random number generator for use across the class
    std::mt19937 random_num_gen_;

    // Passing configuration
    TbotsProto::PassingConfig passing_config_;
};

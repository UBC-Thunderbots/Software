#pragma once

#include <random>

#include "base_pass_generator.h"
#include "proto/parameters.pb.h"
#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass_with_rating.h"
#include "software/world/world.h"

static const int NUM_POINTS_TO_SAMPLE_PER_ROBOT = 5;
static const double SAMPLING_SD_METERS          = 0.1;

class SamplingPassGenerator : public BasePassGenerator
{
   public:
    SamplingPassGenerator(TbotsProto::PassingConfig passing_config);

    virtual PassWithRating getBestPass(const World& world);

   private:
    std::vector<Point> samplePasses(const World& world);

    // A random number generator for use across the class
    std::mt19937 random_num_gen_;

    // Passing configuration
    TbotsProto::PassingConfig passing_config_;
};

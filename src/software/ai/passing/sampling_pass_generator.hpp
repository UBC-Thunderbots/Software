#pragma once

#include "base_pass_generator.hpp"
#include "software/ai/passing/pass_with_rating.h"
#include "software/world/world.h"

static const int NUM_POINTS_TO_SAMPLE_PER_ROBOT = 5;
static const int SAMPLING_SD_METERS = 0.1;

class SamplingPassGenerator: public BasePassGenerator
{
    public:
        explicit SamplingPassGenerator(TbotsProto::PassingConfig passing_config);
    private:
        std::vector<Point> samplePasses(const World& world);

        std::array<Point, NUM_POINTS_TO_SAMPLE_PER_ROBOT> samplePoints(const Robot& robot);

        // A random number generator for use across the class
        std::mt19937 random_num_gen_;

        // Seed generator for the random number generator
        std::random_device seed_generator_;

        // Passing configuration
        TbotsProto::PassingConfig passing_config_;

};

SamplingPassGenerator::SamplingPassGenerator(
    TbotsProto::PassingConfig passing_config)
    : seed_generator_(),
      random_num_gen_(seed_generator_),
      passing_config_(passing_config)
{
}

virtual PassWithRating SamplingPassGenerator::getBestPass(const World& world)
{
    auto sampled_pass_points = samplePasses(world);


}

std::vector<Point> SamplingPassGenerator::samplePasses(const World& world)
{
    std::vector<Point> all_sampled_passes;
    Point pass_start = world.ball().position();

    for (const Robot &robot: world.friendlyTeam().getAllRobots())
    {
        samplePoints(robot, all_sampled_passes);        
    }

    return all_sampled_passes;
}

void SamplingPassGenerator::samplePoints(const Robot& robot, std::vector<Point> sampled_points)
{
    auto robot_position = robot.position();

    std::normal_distribution x_normal_distribution{robot_position.x(), SAMPLING_SD_METERS};
    std::normal_distribution y_normal_distribution{robot_position.y(), SAMPLING_SD_METERS};
    
    for (int i = 0; i < NUM_POINTS_TO_SAMPLE_PER_ROBOT; i++)
    {
        sampled_points.push_back(Point(
            x_normal_distribution(random_num_gen_),
            y_normal_distribution(random_num_gen_)
        ))
    }

    return sampled_points;    
}
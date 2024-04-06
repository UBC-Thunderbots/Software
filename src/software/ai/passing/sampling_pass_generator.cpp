#include "software/ai/passing/sampling_pass_generator.h"

SamplingPassGenerator::SamplingPassGenerator(TbotsProto::PassingConfig passing_config)
    : random_num_gen_(SEED), passing_config_(passing_config)
{
}

PassWithRating SamplingPassGenerator::getBestPass(const World& world)
{
    auto sampled_pass_points = samplePasses(world);

    // if there are no friendly robots, return early
    if (sampled_pass_points.size() == 0)
    {
        // default pass with 0 rating
        return PassWithRating{Pass(Point(), Point(), 0.0), 0};
    }

    std::vector<PassWithRating> sampled_passes_and_ratings;
    sampled_passes_and_ratings.reserve(sampled_pass_points.size());

    // get ratings for each pass
    std::transform(
        sampled_pass_points.begin(), sampled_pass_points.end(),
        sampled_passes_and_ratings.begin(), [&](const Point& point) -> PassWithRating {
            Pass pass = Pass(world.ball().position(), point, 2.0);

            return PassWithRating{pass, ratePass(world, pass, passing_config_)};
        });

    std::sort(sampled_passes_and_ratings.begin(), sampled_passes_and_ratings.end(),
              [](PassWithRating& pass_a, PassWithRating& pass_b) {
                  return pass_a.rating > pass_b.rating;
              });

    // return the highest rated pass
    return sampled_passes_and_ratings.front();
}

std::vector<Point> SamplingPassGenerator::samplePasses(const World& world)
{
    std::vector<Point> all_sampled_passes;

    for (const Robot& robot : world.friendlyTeam().getAllRobots())
    {
        auto robot_position = robot.position();

        // get random coordinates based on the normal distribution around the robot
        std::normal_distribution x_normal_distribution{robot_position.x(),
                                                       SAMPLING_SD_METERS};
        std::normal_distribution y_normal_distribution{robot_position.y(),
                                                       SAMPLING_SD_METERS};

        for (int i = 0; i < NUM_POINTS_TO_SAMPLE_PER_ROBOT; i++)
        {
            all_sampled_passes.push_back(Point(x_normal_distribution(random_num_gen_),
                                               y_normal_distribution(random_num_gen_)));
        }
    }

    return all_sampled_passes;
}

#include "software/ai/passing/sampling_pass_generator.h"

#include <iomanip>

#include "software/logger/logger.h"

SamplingPassGenerator::SamplingPassGenerator(const TbotsProto::PassingConfig& passing_config)
    : random_num_gen_(RNG_SEED), passing_config_(passing_config)
{
}

PassWithRating SamplingPassGenerator::getBestPass(const World& world, const std::vector<RobotId>& robots_to_ignore)
{
    auto sampled_pass_points = sampleReceivingPositions(world, robots_to_ignore);

    // if there are no friendly robots, return early
    if (sampled_pass_points.empty())
    {
        // default pass with 0 rating
        return PassWithRating{Pass(Point(), Point(), 0.0), 0};
    }

    std::vector<PassWithRating> sampled_passes_and_ratings;
    sampled_passes_and_ratings.reserve(sampled_pass_points.size());

    // Get ratings for each pass
    std::transform(sampled_pass_points.begin(), sampled_pass_points.end(),
                   std::back_inserter(sampled_passes_and_ratings),
                   [&](const Point& point) -> PassWithRating {
                       Pass pass = Pass::fromDestReceiveSpeed(world.ball().position(),
                                                              point, passing_config_);

                       double rating = ratePass(world, pass, passing_config_);
                       return PassWithRating{pass, rating};
                   });

    // return the highest rated pass
    auto best_pass = *std::max_element(
        sampled_passes_and_ratings.begin(), sampled_passes_and_ratings.end(),
        [](PassWithRating& pass_a, PassWithRating& pass_b) {
            return pass_a.rating < pass_b.rating;
        });

    std::vector<TbotsProto::DebugShapes::DebugShape>
        debug_shapes;  // TODO (NIMA): Added for debugging
    for (const auto& pass_with_rating : sampled_passes_and_ratings)
    {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(3) << pass_with_rating.rating;
        debug_shapes.push_back(*createDebugShape(
            Circle(pass_with_rating.pass.receiverPoint(), 0.02),
            std::to_string(debug_shapes.size()) + "sp", stream.str()));
    }
    std::stringstream stream;
    stream << "BP:" << std::fixed << std::setprecision(3) << best_pass.rating;
    debug_shapes.push_back(*createDebugShape(
        Circle(best_pass.pass.receiverPoint(), 0.05),
        std::to_string(debug_shapes.size()) + "sp", stream.str()));
    LOG(VISUALIZE) << *createDebugShapes(debug_shapes);

    // Generate sample passes for cost visualization
    if (passing_config_.cost_vis_config().generate_sample_passes())
    {
        samplePassesForVisualization(world, passing_config_, best_pass.pass);
    }

    return best_pass;
}

std::vector<Point> SamplingPassGenerator::sampleReceivingPositions(const World &world, const std::vector<RobotId>& robots_to_ignore)
{
    std::vector<Point> all_sampled_passes;

    const double sampling_std_dev =
            passing_config_.pass_gen_rand_sample_std_dev_meters();

    for (const Robot& robot : world.friendlyTeam().getAllRobots())
    {
        // Ignore robots in the ignore list
        if (std::find(robots_to_ignore.begin(), robots_to_ignore.end(), robot.id()) !=
            robots_to_ignore.end())
        {
            continue;
        }

        // Add the robot's current position to the list of sampled passes
        auto robot_position = robot.position();
        all_sampled_passes.push_back(robot_position);

        // get random coordinates based on the normal distribution around the robot
        std::normal_distribution x_normal_distribution{robot_position.x(),
                                                       sampling_std_dev};
        std::normal_distribution y_normal_distribution{robot_position.y(),
                                                       sampling_std_dev};

        for (unsigned int i = 0; i < passing_config_.pass_gen_num_samples_per_robot();
             i++)
        {
            auto point = Point(x_normal_distribution(random_num_gen_),
                               y_normal_distribution(random_num_gen_));
            all_sampled_passes.push_back(point);
        }
    }

    return all_sampled_passes;
}

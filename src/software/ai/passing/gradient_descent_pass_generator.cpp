#include "software/ai/passing/gradient_descent_pass_generator.h"

#include <iomanip>

#include "software/logger/logger.h"

GradientDescentPassGenerator::GradientDescentPassGenerator(
        const TbotsProto::PassingConfig& passing_config)
        : optimizer_(optimizer_param_weights),
          previous_best_receiving_pos_(Point(0, 0)),
          random_num_gen_(RNG_SEED),
          passing_config_(passing_config)
{
}

PassWithRating GradientDescentPassGenerator::getBestPass(
        const World& world, const std::vector<RobotId>& robots_to_ignore)
{
    num_rate_pass = 0;
    auto receiving_positions = sampleReceivingPositions(world, robots_to_ignore);

    // if there are no friendly robots, return early
    if (receiving_positions.empty())
    {
        // default pass with 0 rating
        return PassWithRating{Pass(Point(), Point(), 0.0), 0};
    }
    receiving_positions.push_back(previous_best_receiving_pos_);

    PassWithRating best_pass = optimizeReceivingPositions(world, receiving_positions);
    previous_best_receiving_pos_ = best_pass.pass.receiverPoint();

    // Visualize the sampled passes and the best pass
    if (passing_config_.pass_gen_vis_config().visualize_sampled_passes())
    {
        std::vector<TbotsProto::DebugShapes::DebugShape> debug_shapes;
        for (const auto& receiving_position : receiving_positions)
        {
            debug_shapes.push_back(*createDebugShape(
                    Circle(receiving_position, 0.02),
                    std::to_string(debug_shapes.size()) + "gdpg"));
        }
        std::stringstream stream;
        stream << "BP:" << std::fixed << std::setprecision(3) << best_pass.rating;
        debug_shapes.push_back(
                *createDebugShape(Circle(best_pass.pass.receiverPoint(), 0.05),
                                  std::to_string(debug_shapes.size()) + "gdpg", stream.str()));
        LOG(VISUALIZE) << *createDebugShapes(debug_shapes);
    }

    // Generate sample passes across the field for cost visualization
    if (passing_config_.cost_vis_config().generate_sample_passes())
    {
        samplePassesForVisualization(world, passing_config_, best_pass.pass);
    }

//    LOG(DEBUG) << "GradientDescentPassGenerator: Number of passes rated: " << num_rate_pass; // TODO (NIMA)

    return best_pass;
}

std::vector<Point> GradientDescentPassGenerator::sampleReceivingPositions(
        const World& world, const std::vector<RobotId>& robots_to_ignore)
{
    std::vector<Point> all_sampled_passes;

    const double sampling_std_dev = passing_config_.pass_gen_rand_sample_std_dev_meters();

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

        for (unsigned int i = 0; i < passing_config_.pass_gen_num_samples_per_robot(); i++)
        {
            auto point = Point(x_normal_distribution(random_num_gen_),
                               y_normal_distribution(random_num_gen_));
            all_sampled_passes.push_back(point);
        }
    }

    return all_sampled_passes;
}

PassWithRating GradientDescentPassGenerator::optimizeReceivingPositions(const World &world,
                                                                        const std::vector<Point> &receiving_positions)
{
    // The objective function we minimize in gradient descent to improve each pass
    // that we're optimizing
    const auto objective_function =
            [this, &world](const std::array<double, NUM_PARAMS_TO_OPTIMIZE>& pass_array) {
                // get a pass with the new appropriate speed using the new destination
                num_rate_pass++;
                return ratePass(
                        world,
                        Pass::fromDestReceiveSpeed(
                                world.ball().position(), Point(pass_array[0], pass_array[1]),
                                passing_config_.max_receive_speed_m_per_s(),
                                passing_config_.min_pass_speed_m_per_s(),
                                passing_config_.max_pass_speed_m_per_s()),
                        passing_config_);
            };

    PassWithRating best_pass{Pass(Point(), Point(), 1.0), 0.0};
    for (const Point& receiving_position : receiving_positions)
    {
        auto optimized_receiving_pos_array = optimizer_.maximize(
                objective_function, {receiving_position.x(), receiving_position.y()},
                passing_config_.number_of_gradient_descent_steps_per_iter());

        // get a pass with the new appropriate speed using the new destination
        auto new_pass = Pass::fromDestReceiveSpeed(
                world.ball().position(),
                Point(optimized_receiving_pos_array[0], optimized_receiving_pos_array[1]),
                passing_config_.max_receive_speed_m_per_s(),
                passing_config_.min_pass_speed_m_per_s(),
                passing_config_.max_pass_speed_m_per_s());
        num_rate_pass++;
        auto score =
                ratePass(world, new_pass, passing_config_);

        if (score > best_pass.rating)
        {
            best_pass = PassWithRating{new_pass, score};
        }
    }

    return best_pass;
}

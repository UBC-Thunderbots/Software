#include "software/ai/passing/cost_function.h"

#include <gtest/gtest.h>

#include <chrono>
#include <random>

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/math/math_functions.h"
#include "software/test_util/test_util.h"

class PassingEvaluationTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        entire_field =
            std::make_shared<Rectangle>(Field::createSSLDivisionBField().fieldLines());
        passing_config         = std::make_shared<PassingConfig>();
        avg_desired_pass_speed = 3.9;
    }

    double avg_desired_pass_speed;

    std::shared_ptr<Rectangle> entire_field;
    std::shared_ptr<const PassingConfig> passing_config;
};

// This test is disabled to speed up CI, it can be enabled by removing "ABLED_" from
// the test name
TEST_F(PassingEvaluationTest, ratePass_speed_test)
{
    // This test does not assert anything. Rather, It can be used to gauge how
    // fast ratePass is running, and can be profiled in order to find areas
    // of improvement for ratePass

    const int num_passes_to_gen = 1000;

    World world = ::TestUtil::createBlankTestingWorld();

    world.updateEnemyTeamState(Team(
        {
            Robot(0, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(1, {1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(2, {0, 1}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(3, {1.5, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(4, {0, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(5, {2.5, -2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(6, {3, -3}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
        },
        Duration::fromSeconds(10)));
    world.updateFriendlyTeamState(Team(
        {
            Robot(0, {-0.2, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(1, {-1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(2, {0, 1}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(3, {-1.5, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(4, {0, -2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(5, {-2.5, -2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(6, {-3, -3}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
        },
        Duration::fromSeconds(10)));

    std::uniform_real_distribution x_distribution(-world.field().xLength() / 2,
                                                  world.field().xLength() / 2);
    std::uniform_real_distribution y_distribution(-world.field().yLength() / 2,
                                                  world.field().yLength() / 2);

    std::uniform_real_distribution speed_distribution(
        passing_config->getMinPassSpeedMPerS()->value(),
        passing_config->getMaxPassSpeedMPerS()->value());

    std::vector<Pass> passes;

    std::mt19937 random_num_gen;
    for (int i = 0; i < num_passes_to_gen; i++)
    {
        Point passer_point(x_distribution(random_num_gen),
                           y_distribution(random_num_gen));
        Point receiver_point(x_distribution(random_num_gen),
                             y_distribution(random_num_gen));
        double pass_speed = speed_distribution(random_num_gen);

        Pass p(passer_point, receiver_point, pass_speed);
        passes.emplace_back(p);
    }

    auto start_time = std::chrono::system_clock::now();
    for (auto pass : passes)
    {
        ratePass(world, pass, *entire_field, passing_config);
    }

    double duration_ms = ::TestUtil::millisecondsSince(start_time);
    double avg_ms      = duration_ms / static_cast<double>(num_passes_to_gen);

    // At the time of this test's creation (PR #695), ratePass ran at an average 0.105ms
    // in debug on an i7
    std::cout << "Took " << duration_ms << "ms to run, average time of " << avg_ms << "ms"
              << std::endl;
}

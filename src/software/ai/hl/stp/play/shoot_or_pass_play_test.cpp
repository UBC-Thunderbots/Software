#include "software/ai/hl/stp/play/shoot_or_pass_play.h"

#include <gtest/gtest.h>

#include <random>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/friendly_scored_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"
std::mt19937 random_num_gen_(PASS_GENERATOR_SEED);

class ShootOrPassPlayTest : public SimulatedPlayTestFixture,
                            public ::testing::WithParamInterface<
                                std::tuple<std::vector<Point>, std::vector<Point>>>
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_F(ShootOrPassPlayTest, test_shoot_or_pass_play)
{
    std::uniform_real_distribution x_distribution(field.enemyHalf().xMin(),
                                                  field.enemyHalf().xMax());
    std::uniform_real_distribution y_distribution(field.enemyHalf().yMin(),
                                                  field.enemyHalf().yMax());
    std::uniform_real_distribution x_other_side_distribution(
        field.friendlyHalf().xMin() + 0.6, field.friendlyHalf().xMax());
    std::uniform_real_distribution y_other_side_distribution(field.friendlyHalf().yMin(),
                                                             field.friendlyHalf().yMax());

    BallState ball_state(Point(-4.4, 2.9), Vector(0, 0));

    for (;;)
    {
        auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({
            field.friendlyGoalCenter(),
            Point(-4.5, 3.0),
            Point(-4, 2.0),
            Point(-3.5, 2.0),
            Point(-2, -2.0),
            Point(-2, -1.5),
        });

        setFriendlyGoalie(0);
        auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
                {Point(-3.5, 2.5), Point(-3.5, 2.7),Point(-3.5, 2.9), Point(-3.5, 2.3), field.enemyGoalCenter(),
             Point(x_distribution(random_num_gen_), y_distribution(random_num_gen_)),
             Point(x_distribution(random_num_gen_), y_distribution(random_num_gen_)),
             Point(x_distribution(random_num_gen_), y_distribution(random_num_gen_)),
             Point(x_distribution(random_num_gen_), y_distribution(random_num_gen_)),
             Point(x_distribution(random_num_gen_), y_distribution(random_num_gen_)),
             Point(x_other_side_distribution(random_num_gen_),
                   y_other_side_distribution(random_num_gen_)),
             Point(x_other_side_distribution(random_num_gen_),
                   y_other_side_distribution(random_num_gen_)),
             Point(x_other_side_distribution(random_num_gen_),
                   y_other_side_distribution(random_num_gen_)),
             Point(x_other_side_distribution(random_num_gen_),
                   y_other_side_distribution(random_num_gen_)),
             Point(x_other_side_distribution(random_num_gen_),
                   y_other_side_distribution(random_num_gen_)),
             field.enemyDefenseArea().negXNegYCorner(),
             field.enemyDefenseArea().negXPosYCorner()});
        setEnemyGoalie(0);
        setAIPlay(TYPENAME(ShootOrPassPlay));
        setRefereeCommand(RefereeCommand::FORCE_START, RefereeCommand::STOP);

        std::vector<ValidationFunction> terminating_validation_functions = {
            [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
                friendlyScored(world_ptr, yield);
            }};

        std::vector<ValidationFunction> non_terminating_validation_functions = {};

        runTest(field, ball_state, friendly_robots, enemy_robots,
                terminating_validation_functions, non_terminating_validation_functions,
                Duration::fromSeconds(30));
    }
}

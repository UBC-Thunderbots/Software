#include "software/ai/hl/stp/play/example_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class ExamplePlayTest : public SimulatedPlayTestFixture
{
};

TEST_F(ExamplePlayTest, test_example_play)
{
    setBallState(BallState(Point(-0.8, 0), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(4, 0), Point(0.5, 0), Point(-3, 1), Point(-1, -3), Point(2, 0),
         Point(3.5, 3)}));
    setFriendlyGoalie(0);
    setAIPlay(TYPENAME(ExamplePlay));

    setRefereeCommand(RefereeCommand::FORCE_START, RefereeCommand::HALT);

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr,
           TerminatingValidationCoroutine::push_type& yield) {
            auto friendly_robots_1_meter_from_ball =
                [](std::shared_ptr<World> world_ptr) {
                    Point ball_position = world_ptr->ball().position();
                    for (const auto& robot : world_ptr->friendlyTeam().getAllRobots())
                    {
                        // Skips the robot assigned as goalie
                        if (robot.id() == 0)
                        {
                            continue;
                        }
                        double abs_error =
                            std::fabs((robot.position() - ball_position).length() - 1.0);
                        if (abs_error > 0.05)
                        {
                            return false;
                        }
                    }
                    return true;
                };

            while (!friendly_robots_1_meter_from_ball(world_ptr))
            {
                yield("Friendly robots not 1 meter away from ball");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(8));
}

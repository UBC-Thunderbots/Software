#include "software/ai/hl/stp/play/offense/offense_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class OffensePlayTest : public SimulatedErForceSimPlayTestFixture
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
};

TEST_F(OffensePlayTest, test_offense_play)
{
    BallState ball_state(Point(-4.4, 2.9), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({
        field.friendlyGoalCenter(),
        Point(-4.5, 3.0),
        Point(-2, 1.5),
        Point(-2, 0.5),
        Point(-2, -1.7),
        Point(-2, -1.5),
    });
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(OffensePlay));
    setRefereeCommand(RefereeCommand::FORCE_START, RefereeCommand::STOP);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1971
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(24.5))
            {
                yield("Timestamp not at 24.5s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(25));
}

TEST_F(OffensePlayTest, test_offense_play_with_keep_away)
{
    BallState ball_state(Point(-1.8, 1.8), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({
        field.friendlyGoalCenter(),
        Point(-4.5, 3.0),
        Point(-2, 1.7),
        Point(-2, 0.5),
        Point(-2, -1.7),
        Point(-2, -1.5),
    });
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-1.5, 1.8), Point(-1.6, 1.95), Point(-1.6, 1.65), Point(-1.4, 1.65),
         field.enemyGoalCenter(), field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(OffensePlay));
    setRefereeCommand(RefereeCommand::FORCE_START, RefereeCommand::STOP);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1971
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(24.5))
            {
                yield("Timestamp not at 24.5s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(25));
}

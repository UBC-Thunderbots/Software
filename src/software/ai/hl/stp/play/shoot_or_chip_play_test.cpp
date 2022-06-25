#include "software/ai/hl/stp/play/shoot_or_chip_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class ShootOrChipPlayTest : public SimulatedErForceSimPlayTestFixture
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
};

TEST_F(ShootOrChipPlayTest, test_shoot_or_chip_play)
{
    BallState ball_state(Point(-1.4, 2), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({
        field.friendlyGoalCenter(),
        Point(-1.5, 2),
        Point(-2, 1.5),
        Point(-2, 0.5),
        Point(-2, -0.5),
        Point(-2, -1.5),
    });
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({
        field.enemyGoalCenter(),
        field.enemyDefenseArea().negXNegYCorner(),
        field.enemyDefenseArea().negXPosYCorner(),
        Point(-1, 0),
        Point(1, -2.5),
    });
    enemy_robots.emplace_back(RobotStateWithId{
        .id          = 5,
        .robot_state = RobotState(Point(1, 2), Vector(-4.6, 0), Angle::half(),
                                  AngularVelocity::zero())});
    setEnemyGoalie(0);
    setAIPlay(TbotsProto::PlayName::ShootOrChipPlay);
    setRefereeCommand(RefereeCommand::FORCE_START, RefereeCommand::STOP);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1971
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
            {
                yield("Timestamp not at 9.5s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

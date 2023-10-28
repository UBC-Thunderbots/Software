#include "software/ai/hl/stp/play/corner_kick_play.h"

#include <gtest/gtest.h>

#include <utility>

#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/friendly_scored_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class CornerKickPlayTest : public SimulatedErForceSimPlayTestFixture
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createFieldProto(field_type);
};

TEST_F(CornerKickPlayTest, test_corner_kick_play_bottom_left)
{
    BallState ball_state(Point(4.5, -3), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(4.6, -3.1)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setAiPlay(TbotsProto::PlayName::CornerKickPlay);
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::INDIRECT_FREE_US);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // TODO (#2612): Re-enable test
        // [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
        //     robotReceivedBall(world_ptr, yield);
        //     friendlyScored(world_ptr, yield);
        // }
    };

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}

TEST_F(CornerKickPlayTest, test_corner_kick_play_top_right)
{
    BallState ball_state(Point(4.5, 3), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, 2.5), Point(0, 1.5), Point(0, 0.5), Point(0, -0.5), Point(0, -1.5),
         Point(4.6, 3.1)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setAiPlay(TbotsProto::PlayName::CornerKickPlay);
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::INDIRECT_FREE_US);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // TODO (#2612): Re-enable test
        //[](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
        //    robotReceivedBall(world_ptr, yield);
        //    friendlyScored(world_ptr, yield);
        //}
    };

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(12));
}

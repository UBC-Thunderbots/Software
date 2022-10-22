#include "software/ai/hl/stp/play/ball_placement_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_at_point_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class BallPlacementPlayTest : public SimulatedErForceSimPlayTestFixture
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
};

TEST_F(BallPlacementPlayTest, test_ball_placement)
{
    Point ball_placement_point(-3, -2);
    BallState ball_state(Point(0, 0.5), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(4.6, -3.1)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setAiPlay(TbotsProto::PlayName::BallPlacementPlay);
    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_US);
    game_state.setBallPlacementPoint(ball_placement_point);
    setGameState(game_state);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, ball_placement_point](std::shared_ptr<World> world_ptr,
                                     ValidationCoroutine::push_type& yield) {
            ballAtPoint(ball_placement_point, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

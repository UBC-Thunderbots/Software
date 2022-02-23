#include "software/ai/hl/stp/tactic/move_goalie_to_goal_line/move_goalie_to_goal_line_tactic.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_er_force_sim_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class MoveGoalieToGoalLineTacticTest
    : public SimulatedErForceSimTacticTestFixture,
      public ::testing::WithParamInterface<RobotStateWithId>
{
   protected:
    FieldType field_type = FieldType::DIV_B;
    Field field          = Field::createField(field_type);
};

TEST_P(MoveGoalieToGoalLineTacticTest, move_to_goal_line_test)
{
    RobotStateWithId robot_state = GetParam();

    std::vector<RobotStateWithId> friendly_robots = {robot_state};
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4, 0)});

    BallState ball_state = BallState(Point(0.5, 0.5), Vector(0, 0));

    auto tactic = std::make_shared<MoveGoalieToGoalLineTactic>();
    setTactic(tactic);
    setFriendlyRobotId(0);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [tactic](std::shared_ptr<World> world_ptr,
                 ValidationCoroutine::push_type& yield) {
            // We check if the robot reaches the goal line center and faces the opponent
            //
            // The tactic should "done" after reaching the goal line and facing the
            // opponent.
            robotAtOrientation(0, world_ptr, Angle::zero(), Angle::fromDegrees(5), yield);
            robotAtPosition(0, world_ptr, world_ptr->field().friendlyGoalCenter(), 0.05,
                            yield);

            while (!tactic->done())
            {
                yield("Move to goal line tactic not done");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

INSTANTIATE_TEST_CASE_P(
    PassEnvironment, MoveGoalieToGoalLineTacticTest,
    ::testing::Values(
        // Robot on friendly half, not facing opponent
        RobotStateWithId{0, RobotState(Point(1, 2), Vector(1, 1), Angle::fromDegrees(180),
                                       Angle::fromDegrees(10))},

        // Robot on enemy goal line, facing opponent, moving
        RobotStateWithId{0, RobotState(Point(4.5, 0), Vector(1, 1), Angle::fromDegrees(0),
                                       Angle::fromDegrees(15))},

        // Robot already at goal line center, facing opponent
        RobotStateWithId{
            0, RobotState(Field::createSSLDivisionBField().friendlyGoalCenter() +
                              Vector(0.1, 0),
                          Vector(0, 0), Angle::fromDegrees(0), Angle::fromDegrees(0))}));

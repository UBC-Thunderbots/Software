#include <gtest/gtest.h>

#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedAttackerVsGoalieTacticTest
    : public SimulatedTacticTestFixture,
      public ::testing::WithParamInterface<std::tuple<RobotStateWithId, RobotStateWithId>>
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_P(SimulatedAttackerVsGoalieTacticTest, attacker_vs_goalie_test)
{
    RobotStateWithId friendly_robot_state = std::get<0>(GetParam());
    RobotStateWithId enemy_robot_state    = std::get<1>(GetParam());

    std::vector<RobotStateWithId> friendly_robots = {friendly_robot_state};
    std::vector<RobotStateWithId> enemy_robots    = {enemy_robot_state};

    BallState ball_state = BallState(Point(0.5, 0.5), Vector(0, 0));

    TbotsProto::AiConfig friendly_ai_config;
    TbotsProto::AiConfig enemy_ai_config;

    auto friendly_tactic = std::make_shared<AttackerTactic>(friendly_ai_config);
    auto enemy_tactic    = std::make_shared<GoalieTactic>(enemy_ai_config);

    setTactic(friendly_tactic, enemy_tactic);
    setBothRobotId(friendly_robot_state.id, enemy_robot_state.id);

    std::vector<ValidationFunction> terminating_validation_functions = {};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

INSTANTIATE_TEST_CASE_P(
    AttckerVsGoalie, SimulatedAttackerVsGoalieTacticTest,
    ::testing::Values(std::make_tuple(
        RobotStateWithId{1, RobotState(Point(1, 2), Vector(1, 1), Angle::fromDegrees(180),
                                       Angle::fromDegrees(10))},
        RobotStateWithId{0, RobotState(Point(0, 0), Vector(1, 1), Angle::fromDegrees(180),
                                       Angle::fromDegrees(10))})));

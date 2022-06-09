#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/friendly_scored_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class AttackerTacticShootGoalTest
    : public SimulatedErForceSimPlayTestFixture,
      // Params: initial ball state, initial robot position, enemy team, expected
      // chip/kick direction
      public ::testing::WithParamInterface<
          std::tuple<BallState, Point, std::vector<RobotStateWithId>>>
{
   protected:
    TbotsProto::AiConfig ai_config;
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
};

TEST_P(AttackerTacticShootGoalTest, attacker_test_shoot_goal)
{
    BallState ball_state      = std::get<0>(GetParam());
    Point initial_robot_point = std::get<1>(GetParam());
    auto enemy_robots         = std::get<2>(GetParam());

    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({initial_robot_point});
    auto tactic = std::make_shared<AttackerTactic>(ai_config);
    // Make it very obvious when we decide to chip
    tactic->updateControlParams(Point(0, field.fieldLines().yMin()));
    setTactic(0, tactic, {TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA});

    std::vector<ValidationFunction> terminating_validation_functions = {
        [tactic](std::shared_ptr<World> world_ptr,
                 ValidationCoroutine::push_type& yield) {
            while (!tactic->done())
            {
                yield("Tactic not done");
            }
        },
        friendlyScored};
    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(6));
}

INSTANTIATE_TEST_CASE_P(
    ShootGoalEnvironment, AttackerTacticShootGoalTest,
    ::testing::Values(
        // enemy goal blocked by enemy robots with enemy threat right
//        std::make_tuple(BallState(Point(2, 1), Vector()), Point(1, 1),
//                        TestUtil::createStationaryRobotStatesWithId(
//                            {Point(2.4, 1), Point(3, 0.4), Point(3, 0.8), Point(3.1, 0.6),
//                             Point(3.1, 1), Point(4.2, 1.2)})),
        // enemy goal blocked by enemy robots with enemy threat left
        std::make_tuple(BallState(Point(2, 1), Vector()), Point(1, 1),
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(1.5, 1), Point(3, 0.4), Point(3, 0.8), Point(3.1, 0.6),
                             Point(3.1, 1), Point(4.2, 1.2)}))
//        // small opening in enemy formation
//        std::make_tuple(BallState(Point(2, 1), Vector()), Point(1, 1),
//                        TestUtil::createStationaryRobotStatesWithId(
//                            {Point(1, 0), Point(3, 0.2), Point(3, 0.8), Point(3.1, 0),
//                             Point(3.1, 1), Point(4.2, 1.2)})),
//        // extreme angle shot
//        std::make_tuple(BallState(Point(3.8, -1.9), Vector()), Point(1, 1),
//                        TestUtil::createStationaryRobotStatesWithId(
//                            {Point(1, 0), Point(3, 1.2), Point(3, 0.8), Point(3.1, 0.6),
//                             Point(3.1, 1), Point(4.2, 0.5)})),
//        // enemy trying to steal
//        std::make_tuple(BallState(Point(2.5, -1), Vector()), Point(1, 1),
//                        TestUtil::createStationaryRobotStatesWithId(
//                            {Point(2.5, -1.4), Point(3, 0.4), Point(3, 0.8),
//                             Point(3.1, 0.6), Point(3.1, 1), Point(4.2, 1.2)}))

                             ));

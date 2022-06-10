#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class AttackerTacticKeepAwayTest
    : public SimulatedErForceSimPlayTestFixture,
      public ::testing::WithParamInterface<std::tuple<Pass, RobotStateWithId, BallState>>
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
};

TEST_P(AttackerTacticKeepAwayTest, attacker_test_passing)
{
    Pass pass                    = std::get<0>(GetParam());
    RobotStateWithId robot_state = std::get<1>(GetParam());
    BallState ball_state         = std::get<2>(GetParam());

    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5)});
    friendly_robots.emplace_back(robot_state);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4, 0)});

    TbotsProto::AiConfig ai_config;
    // force passing for this test by setting min acceptable shot angle very high
    ai_config.mutable_attacker_tactic_config()->set_min_open_angle_for_shot_deg(90);

    auto tactic = std::make_shared<AttackerTactic>(ai_config);
    tactic->updateControlParams(pass, true);
    setTactic(1, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [pass, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            // We check if the robot reaches the desired orientation, at the
            // desired position before checking if the ball has been kicked.
            //
            // The tactic should "done" after kicking the ball.
            robotAtOrientation(1, world_ptr, pass.passerOrientation(),
                               Angle::fromDegrees(5), yield);
            robotAtPosition(1, world_ptr, pass.passerPoint(), 0.1, yield);
            ballKicked(pass.passerOrientation(), world_ptr, yield);

            while (!tactic->done())
            {
                yield("Attacker tactic kicked ball but is not done");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(7));
}

INSTANTIATE_TEST_CASE_P(
    PassEnvironment, AttackerTacticKeepAwayTest,
    ::testing::Values(
        // Stationary Ball Tests
        // Attacker point != Balls location & Balls location != Robots Location
        std::make_tuple(Pass(Point(0.0, 0.5), Point(0, 0), 5),
                        RobotStateWithId{
                            1, RobotState(Point(0, 0), Vector(0, 0),
                                          Angle::fromDegrees(0),
                                          Angle::fromDegrees(0))},
                        BallState(Point(0.5, 0.5), Vector(0, 0))),

        // Attacker point == Balls location & Balls location != Robots Location
        std::make_tuple(Pass(Point(-0.5, -0.5), Point(0, 0), 5),
                        RobotStateWithId{
                            1, RobotState(Point(0, 0), Vector(0, 0),
                                          Angle::fromDegrees(0),
                                          Angle::fromDegrees(0))},
                        BallState(Point(-0.5, -0.5), Vector(0, 0))),

        // Attacker point != Balls location & Balls location == Robots Location
        std::make_tuple(Pass(Point(0.4, 0.4), Point(0, 1), 5),
                        RobotStateWithId{
                            1, RobotState(Point(0.5, 0.5), Vector(0, 0),
                                          Angle::fromDegrees(0),
                                          Angle::fromDegrees(0))},
                        BallState(Point(-0.4, 0.4), Vector(0, 0))),

        // Attacker point == Balls location & Balls location == Robots Location
        std::make_tuple(Pass(Point(0.0, 0.0), Point(0, 0), 5),
                        RobotStateWithId{
                            1, RobotState(Point(0, 0), Vector(0, 0),
                                          Angle::fromDegrees(0),
                                          Angle::fromDegrees(0))},
                        BallState(Point(0.0, 0.0), Vector(0, 0))),

        // Attacker point far away (not a normal use case, but just to sanity check)
        std::make_tuple(Pass(Point(0.0, 0.0), Point(0, 0), 5),
                        RobotStateWithId{
                            1, RobotState(Point(3.5, 2.5), Vector(0, 0),
                                          Angle::fromDegrees(0),
                                          Angle::fromDegrees(0))},
                        BallState(Point(0.0, 0.0), Vector(0, 0))),

        // Attacker point != Balls location & Balls location != Robots Location
        std::make_tuple(Pass(Point(0.0, 0.5), Point(0, 0), 5),
                        RobotStateWithId{
                            1, RobotState(Point(0, 0), Vector(0, 0),
                                          Angle::fromDegrees(0),
                                          Angle::fromDegrees(0))},
                        BallState(Point(0.5, 0.5), Vector(0, 0))),

        // Moving Ball Tests
        // Attacker point == Balls location & Balls location != Robots Location
        std::make_tuple(Pass(Point(-0.5, -0.5), Point(0, 0), 5),
                        RobotStateWithId{
                            1, RobotState(Point(0, 0), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))},
                        BallState(Point(-0.5, -0.5), Vector(1, 0))),

        // Attacker point != Balls location & Balls location == Robots Location
        std::make_tuple(Pass(Point(0.4, 0.4), Point(0, 1), 5),
                        RobotStateWithId{
                            1, RobotState(Point(0.5, 0.5), Vector(0, 0),
                                          Angle::fromDegrees(0),
                                          Angle::fromDegrees(0))},
                        BallState(Point(-0.4, 0.4), Vector(0, 1))),

        // Attacker point == Balls location & Balls location == Robots Location
        std::make_tuple(Pass(Point(0.0, 0.0), Point(0, 0), 5),
                        RobotStateWithId{
                            1, RobotState(Point(0, 0), Vector(0, 0),
                                          Angle::fromDegrees(0),
                                          Angle::fromDegrees(0))},
                        BallState(Point(0.0, 0.0), Vector(1, 0)))
        ));

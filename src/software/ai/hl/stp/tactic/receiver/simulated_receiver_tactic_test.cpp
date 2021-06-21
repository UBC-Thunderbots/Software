#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/friendly_scored_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedReceiverTacticTest
    : public virtual SimulatedTacticTestFixture,
      public ::testing::WithParamInterface<std::tuple<Pass, RobotStateWithId>>
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_P(SimulatedReceiverTacticTest, perfect_pass_receiver_test)
{
    Pass pass                    = std::get<0>(GetParam());
    RobotStateWithId robot_state = std::get<1>(GetParam());
    BallState ball_state =
        BallState(pass.passerPoint(),
                  pass.speed() * (pass.receiverPoint() - pass.passerPoint()).normalize());

    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5)});
    friendly_robots.emplace_back(robot_state);

    auto tactic = std::make_shared<ReceiverTactic>(pass);
    tactic->updateControlParams(pass);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [pass, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            // We check if the robot reaches the desired orientation, at the
            // desired position before checking if the ball has been received.
            //
            // The tactic should "done" after receiving the ball.
            robotAtOrientation(1, world_ptr, pass.receiverOrientation(),
                               Angle::fromDegrees(5), yield);

            // NOTE: we don't check robotAtPosition for receive and dribble
            // because the robot is free to adjust itself to best receive
            // the pass (and dribble). We only care if the robot received the ball.
            robotReceivedBall(1, world_ptr, yield);

            while (!tactic->done())
            {
                yield("Receiver tactic done but did not receive pass");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, {}, terminating_validation_functions,
            non_terminating_validation_functions, Duration::fromSeconds(10));
}

INSTANTIATE_TEST_CASE_P(
    PassEnvironmentReceiveAndDribble, SimulatedReceiverTacticTest,
    ::testing::Values(
        // Robot already at receive point
        std::make_tuple(Pass(Point(0.0, 0.5), Point(2, 2), 3),
                        RobotStateWithId{
                            1, RobotState(Point(2, 2), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))}),

        // Robot slighty off from receive point: test 1
        std::make_tuple(Pass(Point(0.0, 0.5), Point(2, 2), 3),
                        RobotStateWithId{
                            1, RobotState(Point(2, 1.5), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))}),

        // Robot slighty off from receive point: test 2
        std::make_tuple(Pass(Point(0.0, 0.5), Point(2, 2), 3),
                        RobotStateWithId{
                            1, RobotState(Point(2.5, 2.0), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))}),

        // Robot facing away from pass
        std::make_tuple(Pass(Point(0.0, 0.0), Point(-3, 0), 3),
                        RobotStateWithId{1, RobotState(Point(-3, 0), Vector(0, 0),
                                                       Angle::fromDegrees(180),
                                                       Angle::fromDegrees(0))}),

        // Robot facing towards from pass
        std::make_tuple(Pass(Point(0.0, 0.0), Point(-3, 0), 3),
                        RobotStateWithId{
                            1, RobotState(Point(-3, 0), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))}),

        // Robot facing towards pass speedy
        std::make_tuple(Pass(Point(0.0, 0.0), Point(-3, 0), 5),
                        RobotStateWithId{1, RobotState(Point(-3, 0), Vector(0, 0),
                                                       Angle::fromDegrees(0),
                                                       Angle::fromDegrees(0))})));

class SimulatedReceiverTacticTestOneTouch : public SimulatedReceiverTacticTest
{
};

TEST_P(SimulatedReceiverTacticTestOneTouch, test_one_touch)
{
    Pass pass                    = std::get<0>(GetParam());
    RobotStateWithId robot_state = std::get<1>(GetParam());
    BallState ball_state =
        BallState(pass.passerPoint(),
                  pass.speed() * (pass.receiverPoint() - pass.passerPoint()).normalize());

    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5)});
    friendly_robots.emplace_back(robot_state);

    auto tactic = std::make_shared<ReceiverTactic>(pass);
    tactic->updateControlParams(pass);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [pass, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            // We just care if we scored!
            friendlyScored(world_ptr, yield);

            while (!tactic->done())
            {
                yield("Receiver tactic done but did not receive pass");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, {}, terminating_validation_functions,
            non_terminating_validation_functions, Duration::fromSeconds(10));
}

INSTANTIATE_TEST_CASE_P(
    PassEnvironmentOneTouchShot, SimulatedReceiverTacticTestOneTouch,
    ::testing::Values(

        // one touch robot on receiver point
        std::make_tuple(Pass(Point(2.0, 0.0), Point(3.5, 2.5), 3.5),
                        RobotStateWithId{
                            1, RobotState(Point(3.5, 2.5), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))}),

        std::make_tuple(Pass(Point(2.0, 0.0), Point(3.5, -2.5), 3.5),
                        RobotStateWithId{
                            1, RobotState(Point(3.5, -2.5), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))}),

        // one touch robot away from receiver point
        std::make_tuple(Pass(Point(1.5, 0.0), Point(2.5, 2.5), 3.5),
                        RobotStateWithId{
                            1, RobotState(Point(2.0, 2.5), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))}),

        std::make_tuple(Pass(Point(1.5, 0.0), Point(2.5, -2.5), 3.5),
                        RobotStateWithId{
                            1, RobotState(Point(2.0, -2.5), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))}),

        // Sharp angles, these are only a finite set of what
        // sort of sharp angles we can achieve.
        //
        // If we are noticing issues with one-touch on the field, we should
        // add more tests here and explore more of the "one-touch" space
        std::make_tuple(Pass(Point(4.3, -1.5), Point(3, 0), 3.5),
                        RobotStateWithId{
                            1, RobotState(Point(3.0, 0.5), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))}),

        std::make_tuple(Pass(Point(4.0, 1.5), Point(4, -1), 3.5),
                        RobotStateWithId{1, RobotState(Point(4.0, -1), Vector(0, 0),
                                                       Angle::fromDegrees(180),
                                                       Angle::fromDegrees(0))}),

        std::make_tuple(Pass(Point(4.0, 1.5), Point(3.5, -1), 3.5),
                        RobotStateWithId{
                            1, RobotState(Point(3.5, -1), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))}),

        std::make_tuple(Pass(Point(4.0, 1.5), Point(3.0, -1), 3.5),
                        RobotStateWithId{1, RobotState(Point(3.0, -1), Vector(0, 0),
                                                       Angle::fromDegrees(180),
                                                       Angle::fromDegrees(0))}),

        std::make_tuple(Pass(Point(4.3, -1.5), Point(3, 0), 3.5),
                        RobotStateWithId{
                            1, RobotState(Point(3.0, 0.5), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))}),

        std::make_tuple(Pass(Point(4.0, -1.5), Point(4, 1), 3.5),
                        RobotStateWithId{1, RobotState(Point(4.0, 1), Vector(0, 0),
                                                       Angle::fromDegrees(180),
                                                       Angle::fromDegrees(0))}),

        std::make_tuple(Pass(Point(4.0, -1.5), Point(3.5, 1), 3.5),
                        RobotStateWithId{
                            1, RobotState(Point(3.5, 1), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))}),

        std::make_tuple(Pass(Point(4.0, -1.5), Point(3.0, 1), 3.5),
                        RobotStateWithId{
                            1, RobotState(Point(3.0, 1), Vector(0, 0),
                                          Angle::fromDegrees(0), Angle::fromDegrees(0))}),

        // Direct one touch
        std::make_tuple(Pass(Point(3.0, 0.0), Point(2, 0), 5),
                        RobotStateWithId{1, RobotState(Point(2, 0), Vector(0, 0),
                                                       Angle::fromDegrees(0),
                                                       Angle::fromDegrees(0))})));

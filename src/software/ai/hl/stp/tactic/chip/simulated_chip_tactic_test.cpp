#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/chip/chip_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedChipTacticTest
    : public SimulatedTacticTestFixture,
      public ::testing::WithParamInterface<std::tuple<Vector, Angle>>
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_P(SimulatedChipTacticTest, chip_test)
{
    Vector ball_offset_from_robot = std::get<0>(GetParam());
    Angle angle_to_kick_at        = std::get<1>(GetParam());

    Point robot_position = Point(0, 0);
    BallState ball_state(robot_position + ball_offset_from_robot, Vector(0, 0));

    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), robot_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4, 0)});

    auto tactic = std::make_shared<ChipTactic>(false);
    tactic->updateControlParams(robot_position + ball_offset_from_robot, angle_to_kick_at,
                                5);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [angle_to_kick_at, tactic](std::shared_ptr<World> world_ptr,
                                   ValidationCoroutine::push_type& yield) {
            while (!tactic->done())
            {
                yield("Tactic did not complete!");
            }
            ballKicked(angle_to_kick_at, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(5));
}

INSTANTIATE_TEST_CASE_P(BallLocations, SimulatedChipTacticTest,
                        ::testing::Values(
                            // place the ball directly to the left of the robot
                            std::make_tuple(Vector(0, 0.5), Angle::zero()),
                            // place the ball directly to the right of the robot
                            std::make_tuple(Vector(0, -0.5), Angle::zero()),
                            // place the ball directly infront of the robot
                            std::make_tuple(Vector(0.5, 0), Angle::zero()),
                            // place the ball directly behind the robot
                            std::make_tuple(Vector(-0.5, 0), Angle::zero()),
                            // place the ball in the robots dribbler
                            std::make_tuple(Vector(ROBOT_RADIUS, 0), Angle::zero()),

                            // Repeat the same tests but kick in the opposite direction
                            // place the ball directly to the left of the robot
                            std::make_tuple(Vector(0, 0.5), Angle::half()),
                            // place the ball directly to the right of the robot
                            std::make_tuple(Vector(0, -0.5), Angle::half()),
                            // place the ball directly infront of the robot
                            std::make_tuple(Vector(0.5, 0), Angle::half()),
                            // place the ball directly behind the robot
                            std::make_tuple(Vector(-0.5, 0), Angle::half()),
                            // place the ball in the robots dribbler
                            std::make_tuple(Vector(ROBOT_RADIUS, 0), Angle::zero())));

class SimulatedTacticTargetTest : public SimulatedTacticTestFixture,
                                  public ::testing::WithParamInterface<double>
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_P(SimulatedTacticTargetTest, chip_target_test)
{
    double chip_target_distance = GetParam();
    chip_target_distance++;
    Vector ball_offset_from_robot = Vector(0.2, 0);
    Angle angle_to_kick_at        = Angle::fromDegrees(-30);

    Point robot_position = Point(-4, 3);
    BallState ball_state(robot_position + ball_offset_from_robot, Vector(0, 0));

    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), robot_position});

    auto tactic = std::make_shared<ChipTactic>(false);
    tactic->updateControlParams(robot_position + ball_offset_from_robot, angle_to_kick_at,
                                chip_target_distance);
    setTactic(tactic);
    setRobotId(1);

    // TODO (#TODO) Why are we inaccurate in sim?
    const double CHIP_TARGET_ERROR_TOLERANCE  = 0.10;
    const double BALL_MIN_DISTANCE_OFF_GROUND = 0.05;

    std::vector<ValidationFunction> terminating_validation_functions = {
        [&](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->ball().currentState().distanceFromGround() <
                   BALL_MIN_DISTANCE_OFF_GROUND)
            {
                yield("Ball didn't leave the ground!");
            }

            Timestamp chip_start = world_ptr->getMostRecentTimestamp();

            while (world_ptr->ball().currentState().distanceFromGround() >
                   BALL_MIN_DISTANCE_OFF_GROUND)
            {
                yield("Ball didn't stay in the air!");
            }
            while (world_ptr->ball().currentState().distanceFromGround() != 0.0)
            {
                yield("Ball didn't hit the ground! Forgot to turn on gravity again?");
            }

            Duration hang_time = world_ptr->getMostRecentTimestamp() - chip_start;

            // Ball just touched the ground, lets grab the distance it was chipped and
            // check if we are within CHIP_TARGET_ERROR_TOLERANCE meters of where the
            // chip tactic promised it would land.
            //
            // This is testing our chipping simulation.
            double actual_chip_distance =
                ((robot_position + ball_offset_from_robot) - world_ptr->ball().position())
                    .length();
            EXPECT_NEAR(actual_chip_distance, chip_target_distance,
                        CHIP_TARGET_ERROR_TOLERANCE);

            LOG(INFO) << "Ball hang time " << hang_time << " for chip dist "
                      << actual_chip_distance;
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, {}, terminating_validation_functions,
            non_terminating_validation_functions, Duration::fromSeconds(10));
}

INSTANTIATE_TEST_CASE_P(BallLocations, SimulatedTacticTargetTest,
                        testing::Range(2.0, 5.0, 0.5));

#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/assigned_skill/assigned_skill_tactics.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class PivotKickSkillTest : public SimulatedErForceSimPlayTestFixture,
                           public ::testing::WithParamInterface<std::tuple<Vector, Angle>>
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
};

TEST_P(PivotKickSkillTest, pivot_kick_test)
{
    Vector ball_offset_from_robot = std::get<0>(GetParam());
    Angle angle_to_kick_at        = std::get<1>(GetParam());

    Point robot_position = Point(0, 0);
    BallState ball_state(robot_position + ball_offset_from_robot, Vector(0, 0));

    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), robot_position});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4, 0)});

    auto tactic = std::make_shared<PivotKickSkillTactic>(strategy);
    tactic->updateControlParams({robot_position + ball_offset_from_robot,
                                 angle_to_kick_at,
                                 {AutoChipOrKickMode::AUTOKICK, 5}});
    setTactic(1, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [angle_to_kick_at, tactic](std::shared_ptr<World> world_ptr,
                                   ValidationCoroutine::push_type& yield) {
            while (!tactic->done())
            {
                yield("Skill did not complete!");
            }
            ballKicked(angle_to_kick_at, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

INSTANTIATE_TEST_CASE_P(
    BallLocations, PivotKickSkillTest,
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
        std::make_tuple(Vector(ROBOT_MAX_RADIUS_METERS, 0), Angle::zero()),

        // TODO (#2859): The robot does not dribble far enough into the ball
        // Repeat the same tests but kick in the opposite direction
        // place the ball directly to the left of the robot
        // std::make_tuple(Vector(0, 0.5), Angle::half()),

        // place the ball directly to the right of the robot
        std::make_tuple(Vector(0, -0.5), Angle::half()),
        // place the ball directly infront of the robot
        std::make_tuple(Vector(0.5, 0), Angle::half()),

        // place the ball directly behind the robot
        std::make_tuple(Vector(-0.5, 0), Angle::half()),

        // place the ball in the robots dribbler
        std::make_tuple(Vector(ROBOT_MAX_RADIUS_METERS, 0), Angle::zero())));

#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/non_terminating_validation_functions/robot_not_excessively_dribbling_validation.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_at_point_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedDribbleTacticPushEnemyTest : public SimulatedTacticTestFixture,
                                            public ::testing::WithParamInterface<Point>
{
   protected:
    void checkPossession(std::shared_ptr<DribbleTactic> tactic,
                         std::shared_ptr<World> world_ptr,
                         ValidationCoroutine::push_type& yield)
    {
        while (!tactic->done())
        {
            yield("Tactic not done");
        }
        robotReceivedBall(1, world_ptr, yield);
        auto received_ball_time = world_ptr->getMostRecentTimestamp();
        while (world_ptr->getMostRecentTimestamp() <
               received_ball_time + Duration::fromSeconds(1))
        {
            yield("Waiting 1 second to see if possession is maintained");
        }
        robotReceivedBall(1, world_ptr, yield);
    }

    void SetUp() override
    {
        SimulatedTacticTestFixture::SetUp();
        setMotionConstraints({MotionConstraint::ENEMY_DEFENSE_AREA});
    }
    Field field = Field::createSSLDivisionBField();
    std::vector<RobotStateWithId> enemy_robots =
        TestUtil::createStationaryRobotStatesWithId(
            {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
             field.enemyDefenseArea().negXNegYCorner(),
             field.enemyDefenseArea().negXPosYCorner()});
};

TEST_P(SimulatedDribbleTacticPushEnemyTest, test_steal_ball_from_behind_enemy)
{
    Point initial_position = GetParam();
    BallState ball_state(Point(1 + DIST_TO_FRONT_OF_ROBOT_METERS, 2.5), Vector());
    Point dribble_destination = Point(0, 2);
    Angle dribble_orientation = Angle::zero();
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, -2.5), initial_position});

    auto tactic = std::make_shared<DribbleTactic>();
    tactic->updateControlParams(dribble_destination, dribble_orientation);
    setTactic(tactic);
    setRobotId(1);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

INSTANTIATE_TEST_CASE_P(CreaseDefenderEnvironment, SimulatedDribbleTacticPushEnemyTest,
                        ::testing::Values(
                            // Steal ball from behind
                            Point(-2, 2.5),
                            // Steal ball from front
                            Point(3.5, 2.5),
                            // Steal ball from side
                            Point(1, 0)));

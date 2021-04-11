#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedThetaStarOscillationTest : public SimulatedTacticTestFixture
{
};

TEST_F(SimulatedThetaStarOscillationTest, test_theta_star_oscillation)
{
    /*
     * Known destinations that oscillate:
     *  - (-2.5,-2.5)
     *  - (2.5,-2.5)
     *
     * Known destinations that do not oscillate:
     *  - (1, 0)
     *  - (-2.5, 2)
     */
    Point destination      = Point(-2.5, -2.5);
    Point initial_position = Point(-3, 1.5);
    setBallState(BallState(Point(0, 0), Vector(0, 0)));
    addFriendlyRobots(
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position}));
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId({Point(1, 0)}));

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setRobotId(1);

    std::set<MotionConstraint> motion_constraints;
    motion_constraints.insert(MotionConstraint::ENEMY_ROBOTS_COLLISION);
    setMotionConstraints(motion_constraints);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [destination, tactic](std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
            {
                yield("Timestamp not at 9.5s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}

TEST_F(SimulatedThetaStarOscillationTest, test_theta_star_zig_zag_test)
{
    Point destination      = Point(1, 0);
    Point initial_position = Point(-1.6, 0.2);
    setBallState(BallState(Point(0, -0.6), Vector(0, 0)));
    addFriendlyRobots(
            TestUtil::createStationaryRobotStatesWithId({Point(-3, 0), initial_position}));
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId({
        Point(-1, 0.2), Point(-0.5, -0.2), Point(0, 0.2), Point(0.5, -0.2),
        Point(-1, 0.7), Point(-0.7, -0.6), Point(0, 0.7)}));

    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setRobotId(1);

    std::set<MotionConstraint> motion_constraints;
    motion_constraints.insert(MotionConstraint::ENEMY_ROBOTS_COLLISION);
    setMotionConstraints(motion_constraints);

    std::vector<ValidationFunction> terminating_validation_functions = {
            [destination, tactic](std::shared_ptr<World> world_ptr,
                                  ValidationCoroutine::push_type& yield) {
                while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
                {
                    yield("Timestamp not at 9.5s");
                }
            }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}

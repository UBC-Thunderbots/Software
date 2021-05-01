#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_in_polygon_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedCreaseDefenderTacticPositionTest
    : public SimulatedTacticTestFixture,
      public ::testing::WithParamInterface<
          std::tuple<Point, CreaseDefenderAlignment, unsigned int>>
{
};

class SimulatedCreaseDefenderTacticChipTest : public SimulatedTacticTestFixture
{
};

TEST_F(SimulatedCreaseDefenderTacticChipTest, test_chip_ball)
{
    Point enemy_threat_point          = Point(-1.5, 0.5);
    CreaseDefenderAlignment alignment = CreaseDefenderAlignment::CENTRE;

    Point initial_position = Point(-3, 1.5);
    setBallState(BallState(enemy_threat_point, Vector(-2, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId({initial_position}));
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -1.5), field().enemyGoalCenter(),
         field().enemyDefenseArea().negXNegYCorner(),
         field().enemyDefenseArea().negXPosYCorner()}));

    auto robot_navigation_obstacle_config =
        std::make_shared<RobotNavigationObstacleConfig>();

    auto tactic =
        std::make_shared<CreaseDefenderTactic>(robot_navigation_obstacle_config);
    tactic->updateControlParams(enemy_threat_point, alignment);
    setTactic(tactic);
    setRobotId(0);
    setMotionConstraints({MotionConstraint::ENEMY_ROBOTS_COLLISION,
                          MotionConstraint::FRIENDLY_DEFENSE_AREA});

    std::vector<ValidationFunction> terminating_validation_functions = {
        [tactic](std::shared_ptr<World> world_ptr,
                 ValidationCoroutine::push_type& yield) {
            ballKicked(Angle::zero(), world_ptr, yield);
            while (!tactic->done())
            {
                yield("Tactic not done");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_P(SimulatedCreaseDefenderTacticPositionTest, crease_defender_test)
{
    Point enemy_threat_point          = std::get<0>(GetParam());
    CreaseDefenderAlignment alignment = std::get<1>(GetParam());
    unsigned int target_defend_region = std::get<2>(GetParam());
    ASSERT_LE(target_defend_region, 5);

    Point initial_position = Point(-3, 1.5);
    setBallState(BallState(Point(4.5, -3), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId({initial_position}));
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), enemy_threat_point, Point(1, -1.5), field().enemyGoalCenter(),
         field().enemyDefenseArea().negXNegYCorner(),
         field().enemyDefenseArea().negXPosYCorner()}));

    auto robot_navigation_obstacle_config =
        std::make_shared<RobotNavigationObstacleConfig>();

    auto tactic =
        std::make_shared<CreaseDefenderTactic>(robot_navigation_obstacle_config);
    tactic->updateControlParams(enemy_threat_point, alignment);
    setTactic(tactic);
    setRobotId(0);
    setMotionConstraints({MotionConstraint::ENEMY_ROBOTS_COLLISION,
                          MotionConstraint::FRIENDLY_DEFENSE_AREA});

    Rectangle defense_area         = field().friendlyDefenseArea();
    Rectangle field_lines          = field().fieldLines();
    Vector defense_area_half_width = Vector(defense_area.xLength() / 2, 0);

    std::vector<Rectangle> defender_regions = {
        Rectangle(field().friendlyCornerPos(),
                  defense_area.negXPosYCorner() + defense_area_half_width),
        Rectangle(field().friendlyCornerPos() + defense_area_half_width,
                  defense_area.posXPosYCorner()),
        Rectangle(Point(defense_area.xMax(), field_lines.yMax()), field().centerPoint()),
        Rectangle(Point(defense_area.xMax(), field_lines.yMin()), field().centerPoint()),
        Rectangle(field().friendlyCornerNeg() + defense_area_half_width,
                  defense_area.posXNegYCorner()),
        Rectangle(field().friendlyCornerNeg(),
                  defense_area.negXNegYCorner() + defense_area_half_width),
    };

    std::vector<ValidationFunction> terminating_validation_functions = {
        [target_defend_region, defender_regions, tactic](
            std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // Check that tactic is done
            while (!tactic->done())
            {
                yield("Tactic not done");
            }
            // Check that conditions hold for 1000 ticks
            unsigned num_ticks = 1000;
            for (unsigned i = 0; i < num_ticks; i++)
            {
                robotInPolygon(0, defender_regions[target_defend_region], world_ptr,
                               yield);
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

// We check if the robot is in one of the following regions
//  ┌───┬───┬────────────────┐
//  │   │   │                │
//  │ 0 │ 1 │                C
//  │   │   │                E
//  │   │   │                N
//  ├───┴───┤       2        T
//  E       │                R
//  N  Def. │                E
//  D       │                │
//  L       ├────────────────┤
//  I  Area │                │
//  N       │                │
//  E       │                L
//  ├───┬───┤                I
//  │   │   │       3        N
//  │ 5 │ 4 │                E
//  │   │   │                │
//  │   │   │                │
//  └───┴───┴────────────────┘

INSTANTIATE_TEST_CASE_P(
    CreaseDefenderEnvironment, SimulatedCreaseDefenderTacticPositionTest,
    ::testing::Values(
        // Enemy threat in front of crease, LEFT
        std::make_tuple(Point(1, 2.5), CreaseDefenderAlignment::LEFT, 2),
        // Enemy threat in front of crease, CENTRE
        std::make_tuple(Point(1, -2.5), CreaseDefenderAlignment::CENTRE, 3),
        // Enemy threat in front of crease, RIGHT
        std::make_tuple(Point(1.5, 2), CreaseDefenderAlignment::RIGHT, 2),
        // Enemy threat left side of crease, RIGHT
        std::make_tuple(Point(-3.5, 2.5), CreaseDefenderAlignment::RIGHT, 1),
        // Enemy threat left side of crease, CENTRE
        std::make_tuple(Point(-4, 2.5), CreaseDefenderAlignment::CENTRE, 0),
        // Enemy threat right side of crease, RIGHT
        std::make_tuple(Point(-4, -2), CreaseDefenderAlignment::RIGHT, 5),
        // Enemy threat right side of crease, LEFT
        std::make_tuple(Point(-4.25, -2), CreaseDefenderAlignment::LEFT, 5)));

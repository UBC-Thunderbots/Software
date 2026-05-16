#include "software/ai/hl/stp/play/kickoff_enemy/kickoff_enemy_play.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/calculate_block_cone.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/cpp_validation/validation_function.h"
#include "software/simulated_tests/non_terminating_validation_functions/robots_in_friendly_half_validation.h"
#include "software/simulated_tests/non_terminating_validation_functions/robots_not_in_center_circle_validation.h"
#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_in_polygon_validation.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class KickoffEnemyPlayTest : public SimulatedErForceSimPlayTestFixture
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
};

// TODO (#3105): Re-enable test once destinations are moved outside of obstacles
TEST_F(KickoffEnemyPlayTest, DISABLED_test_kickoff_enemy_play)
{
    BallState ball_state(Point(0, 0), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-field.xLength() / 3.0, 5.0 * field.yLength() / 12.0),
         Point(-field.xLength() / 3.0, field.yLength() / 4.0),
         Point(-field.xLength() / 3.0, field.yLength() / 12.0),
         Point(-field.xLength() / 3.0, -field.yLength() / 12.0),
         Point(-field.xLength() / 3.0, -field.yLength() / 4.0),
         Point(-field.xLength() / 3.0, -5.0 * field.yLength() / 12.0)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(field.xLength() / 9.0, 0),
         Point(field.xLength() / 9.0, 5.0 * field.yLength() / 12.0),
         Point(field.xLength() / 9.0, -5.0 * field.yLength() / 12.0),
         field.enemyGoalCenter(), field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setAiPlay(TbotsProto::PlayName::KickoffEnemyPlay);
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_KICKOFF_THEM);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield)
        {
            const Field test_field = world_ptr->field();
            // Two friendly robots in position to shadow enemy robots. Rectangles are
            // chosen to be generally in the way of the the front 3 enemy robots and the
            // friendly goal, based on where the enemy robots are initialized in the test.
            Rectangle shadowing_rect_1(
                Point(0.0, 1.5 * test_field.yLength() / 6.0),
                Point(-0.4 * test_field.xLength() / 9.0,
                      1.3 * test_field.yLength() / 6.0));
            Rectangle shadowing_rect_2(
                Point(0.0, -1.5 * test_field.yLength() / 6.0),
                Point(-0.4 * test_field.xLength() / 9.0,
                      -1.3 * test_field.yLength() / 6.0));
            Rectangle shadowing_rect_3(
                Point(-0.60 * test_field.xLength() / 9.0,
                      0.1 * test_field.yLength() / 6.0),
                Point(-0.86 * test_field.xLength() / 9.0,
                      -0.1 * test_field.yLength() / 6.0));
            robotInPolygon(shadowing_rect_1, 1, world_ptr, yield);
            robotInPolygon(shadowing_rect_2, 1, world_ptr, yield);
            robotInPolygon(shadowing_rect_3, 1, world_ptr, yield);

            // Two Friendly robots defending the exterior of defense box
            Rectangle robotsDefendingRect(
                Point(test_field.friendlyDefenseArea().xMax() +
                          0.3 * test_field.xLength() / 9.0,
                      1.1 * test_field.yLength() / 6.0),
                Point(test_field.friendlyDefenseArea().xMax(),
                      -1.1 * test_field.yLength() / 6.0));
            robotInPolygon(robotsDefendingRect, 2, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        robotsInFriendlyHalf, robotsNotInCenterCircle};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

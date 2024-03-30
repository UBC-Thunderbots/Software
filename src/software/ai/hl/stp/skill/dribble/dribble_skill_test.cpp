#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/assigned_skill/named_skill_tactics.h"
#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/non_terminating_validation_functions/robot_not_excessively_dribbling_validation.h"
#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_at_point_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class DribbleSkillTest : public SimulatedErForceSimPlayTestFixture
{
   protected:
    void checkPossession(std::shared_ptr<DribbleSkillTactic> tactic,
                         std::shared_ptr<World> world_ptr,
                         ValidationCoroutine::push_type& yield)
    {
        while (!tactic->done())
        {
            yield("Skill not done");
        }
        robotReceivedBall(world_ptr, yield);
        auto received_ball_time = world_ptr->getMostRecentTimestamp();
        while (world_ptr->getMostRecentTimestamp() <
               received_ball_time + Duration::fromSeconds(2))
        {
            yield("Waiting 2 second to see if possession is maintained");
        }
        robotReceivedBall(world_ptr, yield);
    }

    void SetUp() override
    {
        SimulatedErForceSimPlayTestFixture::SetUp();
    }
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
    std::vector<RobotStateWithId> enemy_robots =
        TestUtil::createStationaryRobotStatesWithId(
            {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
             field.enemyDefenseArea().negXNegYCorner(),
             field.enemyDefenseArea().negXPosYCorner()});
    std::shared_ptr<Strategy> strategy =
        std::make_shared<Strategy>(TbotsProto::AiConfig());
    std::set<TbotsProto::MotionConstraint> motion_constraints = {
        TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA};
};

TEST_F(DribbleSkillTest, test_intercept_ball_behind_enemy_robot)
{
    Point initial_position = Point(-3, 1.5);
    BallState ball_state(Point(3, -2), Vector(-0.5, 1));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});

    auto tactic = std::make_shared<DribbleSkillTactic>(strategy);
    setTactic(1, tactic, motion_constraints);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(DribbleSkillTest, test_stopped_ball)
{
    Point initial_position = Point(-3, 1.5);
    BallState ball_state(Point(-1, 1.5), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(3, 3), initial_position});

    auto tactic = std::make_shared<DribbleSkillTactic>(strategy);
    setTactic(1, tactic, motion_constraints);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(DribbleSkillTest, test_ball_bounce_off_of_enemy_robot)
{
    Point initial_position = Point(-3, 1.5);
    BallState ball_state(Point(0, 0), Vector(2.5, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(3, 3), initial_position});

    auto tactic = std::make_shared<DribbleSkillTactic>(strategy);
    setTactic(1, tactic, motion_constraints);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(DribbleSkillTest, test_moving_ball_dribble_dest)
{
    Point initial_position    = Point(-3, 1.5);
    Point dribble_destination = Point(-3, 1);
    BallState ball_state(Point(3, -2), Vector(-1, 2));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});

    auto tactic = std::make_shared<DribbleSkillTactic>(strategy);
    tactic->updateControlParams({dribble_destination, std::nullopt, false});
    setTactic(1, tactic, motion_constraints);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, dribble_destination, tactic](std::shared_ptr<World> world_ptr,
                                            ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
            ballAtPoint(dribble_destination, world_ptr, yield);
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [this](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // TODO (#2514): tune dribbling and re-enable
            // robotNotExcessivelyDribbling(1, world_ptr, yield);
        }};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(15));
}

TEST_F(DribbleSkillTest, test_moving_ball_dribble_orientation)
{
    Point initial_position    = Point(-3, 1.5);
    Angle dribble_orientation = Angle::quarter();
    BallState ball_state(Point(3, -2), Vector(-1, 2));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});

    auto tactic = std::make_shared<DribbleSkillTactic>(strategy);
    tactic->updateControlParams({std::nullopt, dribble_orientation, false});
    setTactic(1, tactic, motion_constraints);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, dribble_orientation, tactic](std::shared_ptr<World> world_ptr,
                                            ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
            robotAtOrientation(1, world_ptr, dribble_orientation, Angle::fromDegrees(5),
                               yield);
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(DribbleSkillTest, test_moving_ball_dribble_dest_and_orientation)
{
    Point initial_position    = Point(-2, 1.5);
    Point dribble_destination = Point(-1, 2);
    Angle dribble_orientation = Angle::zero();
    BallState ball_state(Point(1, 0), Vector(1, 2));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});

    auto tactic = std::make_shared<DribbleSkillTactic>(strategy);
    tactic->updateControlParams({dribble_destination, dribble_orientation, false});
    setTactic(1, tactic, motion_constraints);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, dribble_destination, dribble_orientation, tactic](
            std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
            ballAtPoint(dribble_destination, world_ptr, yield);
            robotAtOrientation(1, world_ptr, dribble_orientation, Angle::fromDegrees(5),
                               yield);
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [this](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // TODO (#2514): tune dribbling and re-enable
            // robotNotExcessivelyDribbling(1, world_ptr, yield);
        }};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(22));
}

TEST_F(DribbleSkillTest, test_dribble_dest_and_orientation_around_rectangle)
{
    Point initial_position    = Point(3, -3);
    Point dribble_destination = Point(4, 2.5);
    Angle dribble_orientation = Angle::half();
    BallState ball_state(Point(4, -2.5), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});

    auto tactic = std::make_shared<DribbleSkillTactic>(strategy);
    tactic->updateControlParams({dribble_destination, dribble_orientation, false});
    setTactic(1, tactic, motion_constraints);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, dribble_destination, dribble_orientation, tactic](
            std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
            ballAtPoint(dribble_destination, world_ptr, yield);
            robotAtOrientation(1, world_ptr, dribble_orientation, Angle::fromDegrees(5),
                               yield);
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [this](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // TODO (#2514): tune dribbling and re-enable
            // robotNotExcessivelyDribbling(1, world_ptr, yield);
        }};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(25));
}

TEST_F(DribbleSkillTest,
       test_dribble_dest_and_orientation_around_rectangle_with_excessive_dribbling)
{
    Point dribble_destination = Point(3, 2);
    Point initial_position    = Point(4.5, -3.0);
    Angle dribble_orientation = Angle::half();
    BallState ball_state(Point(4.2, -2.5), Vector(0, 0));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});

    auto tactic = std::make_shared<DribbleSkillTactic>(strategy);
    tactic->updateControlParams({dribble_destination, dribble_orientation, true});
    setTactic(1, tactic, motion_constraints);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, dribble_destination, dribble_orientation, tactic](
            std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
            ballAtPoint(dribble_destination, world_ptr, yield);
            robotAtOrientation(1, world_ptr, dribble_orientation, Angle::fromDegrees(5),
                               yield);
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(12));
}

TEST_F(DribbleSkillTest, test_running_into_enemy_robot_knocking_ball_away)
{
    Point initial_position    = Point(-2, 1.5);
    Point dribble_destination = Point(-1, 2);
    Angle dribble_orientation = Angle::half();
    BallState ball_state(Point(2, -2), Vector(2, 4));
    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position});
    enemy_robots.emplace_back(RobotStateWithId{
        .id          = 7,
        .robot_state = RobotState(Point(1, 1.1), Vector(), Angle::fromDegrees(-30),
                                  AngularVelocity::zero())});

    auto tactic = std::make_shared<DribbleSkillTactic>(strategy);
    tactic->updateControlParams({dribble_destination, dribble_orientation, false});
    // Don't avoid enemy robots to knock ball away
    setTactic(1, tactic, {});

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, dribble_destination, dribble_orientation, tactic](
            std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            checkPossession(tactic, world_ptr, yield);
            ballAtPoint(dribble_destination, world_ptr, yield);
            robotAtOrientation(1, world_ptr, dribble_orientation, Angle::fromDegrees(5),
                               yield);
            checkPossession(tactic, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [this](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // TODO (#2514): tune dribbling and re-enable
            // robotNotExcessivelyDribbling(1, world_ptr, yield);
        }};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(20));
}

#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/geom/triangle.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_in_polygon_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedShadowEnemyTacticTest : public SimulatedTacticTestFixture
{
    void SetUp() override
    {
        SimulatedTacticTestFixture::SetUp();
        setMotionConstraints({MotionConstraint::ENEMY_DEFENSE_AREA});
    }

   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_F(SimulatedShadowEnemyTacticTest, test_block_pass)
{
    Robot shadower(0, Point(-2, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                   Timestamp::fromSeconds(0));
    Robot shadowee(1, Point(0, -2), Vector(0, 0), Angle::quarter(),
                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy(2, Point(0, 2), Vector(0, 0), Angle::threeQuarter(),
                AngularVelocity::zero(), Timestamp::fromSeconds(0));



    EnemyThreat enemy_threat{shadowee,     false, Angle::zero(), std::nullopt,
                             std::nullopt, 1,     enemy};

    auto friendly_robots = {
        RobotStateWithId{.id = 0, .robot_state = shadower.currentState()}};
    auto enemy_robots = {
        RobotStateWithId{.id = 1, .robot_state = shadowee.currentState()},
        RobotStateWithId{.id = 2, .robot_state = enemy.currentState()}};


    BallState ball_state(Point(0, 2), Vector(0, 0));
    auto tactic = std::make_shared<ShadowEnemyTactic>();
    tactic->updateControlParams(enemy_threat, 2);
    setTactic(tactic);
    setRobotId(0);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            // As the shadowee is located at (0,-2) and the enemy robot that
            // has the ball is located at (0,2), we would like to block the pass
            // with a shadow distance of 2
            Point destination = Point(0, 0);
            robotAtPosition(0, world_ptr, destination, 0.01, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(5));
}

TEST_F(SimulatedShadowEnemyTacticTest, test_block_pass_if_enemy_does_not_have_ball)
{
    Robot shadower(0, Point(-2, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                   Timestamp::fromSeconds(0));
    Robot shadowee(1, Point(0, -2), Vector(0, 0), Angle::quarter(),
                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy(2, Point(0, 2), Vector(0, 0), Angle::threeQuarter(),
                AngularVelocity::zero(), Timestamp::fromSeconds(0));



    EnemyThreat enemy_threat{shadowee,     false, Angle::zero(), std::nullopt,
                             std::nullopt, 1,     enemy};

    auto friendly_robots = {
        RobotStateWithId{.id = 0, .robot_state = shadower.currentState()}};
    auto enemy_robots = {
        RobotStateWithId{.id = 1, .robot_state = shadowee.currentState()},
        RobotStateWithId{.id = 2, .robot_state = enemy.currentState()}};


    BallState ball_state(Point(3, 0), Vector(0, 0));
    auto tactic = std::make_shared<ShadowEnemyTactic>();
    tactic->updateControlParams(enemy_threat, 1.5);
    setTactic(tactic);
    setRobotId(0);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, tactic, shadowee](std::shared_ptr<World> world_ptr,
                                 ValidationCoroutine::push_type& yield) {
            // As the shadowee is located at (0,-2) and the ball is located at (3,0),
            // we would like to block the pass with a shadow distance of 1.5
            Vector pass       = Point(3, 0) - shadowee.position();
            Point destination = shadowee.position() + pass.normalize(1.5);
            robotAtPosition(0, world_ptr, destination, 0.01, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(5));
}

TEST_F(SimulatedShadowEnemyTacticTest, test_block_net_then_steal_and_chip)
{
    Robot shadower(0, Point(-2, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                   Timestamp::fromSeconds(0));
    Robot shadowee(1, Point(0, -2), Vector(0, 0), Angle::fromDegrees(135),
                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy(2, Point(0, 2), Vector(0, 0), Angle::threeQuarter(),
                AngularVelocity::zero(), Timestamp::fromSeconds(0));


    EnemyThreat enemy_threat{shadowee,     true, Angle::zero(), std::nullopt,
                             std::nullopt, 1,    enemy};

    auto friendly_robots = {
        RobotStateWithId{.id = 0, .robot_state = shadower.currentState()}};
    auto enemy_robots = {
        RobotStateWithId{.id = 1, .robot_state = shadowee.currentState()},
        RobotStateWithId{.id = 2, .robot_state = enemy.currentState()}};


    BallState ball_state(Point(0, -1.75), Vector(0, 0));
    auto tactic = std::make_shared<ShadowEnemyTactic>();
    tactic->updateControlParams(enemy_threat, 2);
    setTactic(tactic);
    setRobotId(0);



    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            // We compose a triangle consisting of the friendly goal posts
            // and the ball position. If our robot is in this triangle, then
            // it is blocking a possible shot on net
            Triangle shotTriangle{world_ptr->field().friendlyGoalpostPos(),
                                  world_ptr->field().friendlyGoalpostNeg(),
                                  world_ptr->ball().position()};
            robotInPolygon(0, shotTriangle, world_ptr, yield);
        },
        [this, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            // As our friendly robot tries to steal and chip the ball,
            // it should chip the ball in the same direction is it
            // heading towards the ball
            Vector chip = world_ptr->ball().position() -
                          world_ptr->friendlyTeam().getRobotById(0).value().position();
            ballKicked(chip.orientation(), world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(5));
}

TEST_F(SimulatedShadowEnemyTacticTest, test_block_net_if_enemy_threat_is_null)
{
    Robot shadower(0, Point(-2, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                   Timestamp::fromSeconds(0));
    Robot shadowee(1, Point(0, -2), Vector(0, 0), Angle::fromDegrees(135),
                   AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy(2, Point(0, 2), Vector(0, 0), Angle::threeQuarter(),
                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    auto friendly_robots = {
        RobotStateWithId{.id = 0, .robot_state = shadower.currentState()}};
    auto enemy_robots = {
        RobotStateWithId{.id = 1, .robot_state = shadowee.currentState()},
        RobotStateWithId{.id = 2, .robot_state = enemy.currentState()}};


    BallState ball_state(Point(0, -1.75), Vector(0, 0));
    auto tactic = std::make_shared<ShadowEnemyTactic>();
    tactic->updateControlParams(std::nullopt, 2);
    setTactic(tactic);
    setRobotId(0);



    std::vector<ValidationFunction> terminating_validation_functions = {
        [this, tactic](std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield) {
            // We compose a triangle consisting of the friendly goal posts
            // and the ball position. If our robot is in this triangle, then
            // it is blocking a possible shot on net
            Triangle shotTriangle{world_ptr->field().friendlyGoalpostPos(),
                                  world_ptr->field().friendlyGoalpostNeg(),
                                  world_ptr->ball().position()};
            robotInPolygon(0, shotTriangle, world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(5));
}

#include <gtest/gtest.h>

#include <utility>

#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/geom/algorithms/contains.h"
// #include "software/simulated_tests/non_terminating_validation_functions/robot_not_excessively_dribbling_validation.h"
#include "software/simulated_tests/simulated_tactic_test_fixture.h"
// #include "software/simulated_tests/terminating_validation_functions/ball_at_point_validation.h"
// #include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
// #include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class SimulatedShadowEnemyTacticTest : public SimulatedTacticTestFixture
{

    void SetUp() override
    {
        SimulatedTacticTestFixture::SetUp();
        setMotionConstraints({MotionConstraint::ENEMY_ROBOTS_COLLISION,
                              MotionConstraint::ENEMY_DEFENSE_AREA});
    }
};

TEST_F(SimulatedShadowEnemyTacticTest, test_block_pass)
{
    Robot enemy(0, Point(0, 2), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    Robot shadowee (1, Point(0, -2), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
            Timestamp::fromSeconds(0));
    Robot shadower (2, Point(-2, 0),Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
        Timestamp::fromSeconds(0));
    
    EnemyThreat enemy_threat{shadowee,     false, Angle::zero(), std::nullopt,
                            std::nullopt, 1, enemy};

    setBallState(BallState(Point(0,2), Vector(0,0)));
    auto tactic = std::make_shared<ShadowEnemyTactic>();
    tactic -> updateControlParams(enemy_threat, 2 ,0);
    setTactic(tactic);
   

    std::vector<ValidationFunction> terminating_validation_functions = {
    [this, tactic](std::shared_ptr<World> world_ptr,
                    ValidationCoroutine::push_type& yield) {
        robotAtPosition(2, world_ptr, Point(0,0), 0, yield);
    }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));

}
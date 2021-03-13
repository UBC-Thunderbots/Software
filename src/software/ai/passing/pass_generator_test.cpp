#include "software/ai/passing/pass_generator.h"

#include <gtest/gtest.h>
#include <string.h>

#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/geom/algorithms/contains.h"
#include "software/test_util/test_util.h"

class PassGeneratorTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        passing_config = std::make_shared<const PassingConfig>();
        pitch_division = std::make_shared<const EighteenZonePitchDivision>(world.field());
        pass_generator = std::make_shared<PassGenerator<EighteenZoneId>>(pitch_division,
                                                                         passing_config);
    }

    /**
     * Calls generatePassEvaluation to step the pass generator a couple times 
     * to find better passes.
     *
     * The pass generator starts with bad passes and improves on them as time goes on
     *
     * @param world The world to evaluate passes on
     * @param max_iters The maximum number of iterations of the PassGenerator to run
     */
    static void stepPassGenerator(
        std::shared_ptr<PassGenerator<EighteenZoneId>> pass_generator, const World& world,
        int max_iters)
    {
        for (int i = 0; i < max_iters; i++)
        {
            auto pass_eval = pass_generator->generatePassEvaluation(world);
        }
    }

    World world = ::TestUtil::createBlankTestingWorld();
    std::shared_ptr<const PassingConfig> passing_config;
    std::shared_ptr<const FieldPitchDivision<EighteenZoneId>> pitch_division;
    std::shared_ptr<PassGenerator<EighteenZoneId>> pass_generator;
};

TEST_F(PassGeneratorTest, check_pass_converges)
{
    // Test that we can converge to a stable pass in a scenario where there is a
    // fairly clear best pass.

    world.updateBall(
        Ball(BallState(Point(2, 2), Vector(0, 0)), Timestamp::fromSeconds(0)));
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(3, {1, 0}, {0.5, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, world.field().enemyGoalpostNeg(), {0, 0}, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(1, world.field().enemyGoalpostNeg() - Vector(0.1, 0), {0, 0}, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(2, world.field().enemyGoalpostNeg() - Vector(0.2, 0), {0, 0}, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(3, world.field().enemyGoalpostPos(), {0, 0}, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(4, world.field().enemyGoalpostPos() - Vector(0.1, 0), {0, 0}, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(5, world.field().enemyGoalpostPos() - Vector(0.2, 0), {0, 0}, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(6, {-1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(7, {-1, 0.5}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(8, {-1, -0.5}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateEnemyTeamState(enemy_team);

    stepPassGenerator(pass_generator, world, 100);

    auto [best_pass, score] =
        pass_generator->generatePassEvaluation(world).getBestPassOnField();

    // After 100 iterations on the same world, we should "converge"
    // to the same pass.
    for (int i = 0; i < 7; i++)
    {
        auto [pass, score] =
            pass_generator->generatePassEvaluation(world).getBestPassOnField();

        EXPECT_LE((best_pass.receiverPoint() - pass.receiverPoint()).length(), 0.7);
        EXPECT_LE(abs(best_pass.speed() - pass.speed()), 0.7);
        UNUSED(score);
    }
    UNUSED(score);
}

#include "software/ai/passing/pass_generator.hpp"

#include <gtest/gtest.h>
#include <string.h>

#include "software//world/world.h"
#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/geom/algorithms/contains.h"
#include "software/test_util/test_util.h"

class PassGeneratorTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        passing_config.set_min_pass_speed_m_per_s(1);
        passing_config.set_max_pass_speed_m_per_s(5.5);

        pitch_division = std::make_shared<const EighteenZonePitchDivision>(world.field());
        pass_generator = std::make_shared<PassGenerator<EighteenZoneId>>(pitch_division,
                                                                         passing_config);
    }

    /**
     * Calls generatePassEvaluation to step the pass generator a couple times
     * to find better passes.
     *
     * The pass generator starts with bad passes and improves on them as it receives
     * more "world" inputs
     *
     * @param pass_generator The pass generator to step
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
    TbotsProto::PassingConfig passing_config;
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

    // call generate evaluation 100 times on the given world
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

TEST_F(PassGeneratorTest, check_pass_does_not_converge_to_self_pass)
{
    // Test that we do not converge to a pass from the passer robot to itself

    world.updateBall(Ball(BallState({3.5, 0}, {0, 0}), Timestamp::fromSeconds(0)));

    // The passer robot
    Robot passer = Robot(0, {3.7, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));

    // The potential receiver robot. Not in a great position, but the only friendly on
    // the field
    Robot receiver = Robot(1, {3.7, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                           Timestamp::fromSeconds(0));

    Team friendly_team({passer, receiver}, Duration::fromSeconds(10));
    world.updateFriendlyTeamState(friendly_team);

    // We put a few enemies in to force the pass generator to make a decision,
    // otherwise most of the field would be a valid point to pass to
    Team enemy_team(
        {
            Robot(0, {0, 3}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(1, {0, -3}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
            Robot(2, {2, 3}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0)),
        },
        Duration::fromSeconds(10));
    world.updateEnemyTeamState(enemy_team);

    // call generate evaluation 100 times on the given world
    stepPassGenerator(pass_generator, world, 100);

    // Find what pass we converged to
    auto pass_eval = pass_generator->generatePassEvaluation(world);
    auto [converged_pass, converged_score] = pass_eval.getBestPassOnField();

    std::cout << "RECEIVER POINT: " << converged_pass.receiverPoint() << std::endl;

    // We expect to have converged to a point near robot 2. The tolerance is fairly
    // generous here because the enemies on the field can "force" the point slightly
    // away from the chosen receiver robot
    EXPECT_LE((converged_pass.receiverPoint() - receiver.position()).length(), 0.55);
    UNUSED(converged_score);
}

TEST_F(PassGeneratorTest, test_passer_point_changes_are_respected)
{
    // Test that changing the passer point is reflected in the optimized passes returned

    // Put a friendly robot on the +y and -y sides of the field, both on the enemy half
    Team friendly_team(Duration::fromSeconds(10));
    Robot pos_y_friendly = Robot(0, {2, 2}, {0, 0}, Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot neg_y_friendly = Robot(1, {2, -2}, {0, 0}, Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    friendly_team.updateRobots({pos_y_friendly, neg_y_friendly});
    world.updateFriendlyTeamState(friendly_team);

    // Put a line of enemies along the +x axis, "separating" the two friendly robots
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {0.5, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(2, {1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(3, {1.5, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(4, {2, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(5, {2.5, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(6, {3, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(7, {3.5, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateEnemyTeamState(enemy_team);

    world.updateBall(
        Ball(BallState(Point(3, 1), Vector(0, 0)), Timestamp::fromSeconds(0)));

    // call generate evaluation 100 times on the given world
    stepPassGenerator(pass_generator, world, 100);

    // Find what pass we converged to
    auto pass_evaluation = pass_generator->generatePassEvaluation(world);
    auto converged_pass  = pass_evaluation.getBestPassOnField().pass;

    // We expect to have converged to a point closer to the robot in the neg_y
    // compared to the robot in the pos_y position.
    EXPECT_GT((converged_pass.receiverPoint() - pos_y_friendly.position()).length(),
              (converged_pass.receiverPoint() - neg_y_friendly.position()).length());

    // Set the passer point so that the only reasonable pass is to the robot
    // on the -y side
    world.updateBall(
        Ball(BallState(Point(3, -1), Vector(0, 0)), Timestamp::fromSeconds(0)));

    // call generate evaluation 100 times on the given world
    stepPassGenerator(pass_generator, world, 100);

    // Find what pass we converged to
    pass_evaluation = pass_generator->generatePassEvaluation(world);
    converged_pass  = pass_evaluation.getBestPassOnField().pass;

    // We expect to have converged to a point closer to the robot in the pos_y
    // compared to the robot in the neg_y position.
    EXPECT_GT((converged_pass.receiverPoint() - neg_y_friendly.position()).length(),
              (converged_pass.receiverPoint() - pos_y_friendly.position()).length());
}

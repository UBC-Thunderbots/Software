#include "software/ai/passing/pass_generator.h"

#include <gtest/gtest.h>
#include <string.h>

#include "software/ai/passing/cost_function.h"
#include "software/geom/algorithms/contains.h"
#include "software/test_util/test_util.h"

class PassGeneratorTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        passing_config = std::make_shared<const PassingConfig>();
        pass_generator =
            std::make_shared<PassGenerator>(world, Point(0, 0), PassType::ONE_TOUCH_SHOT,
                                            passing_config, true);
    }

    /**
     * Wait for the pass generator to converge to a pass
     *
     * If the pass generator does not converge within `max_num_seconds`, this will
     * cause the test to fail
     *
     * @param pass_generator Modified in-place
     * @param min_score_diff The minimum difference between two iterations of the pass
     *                       generator below which we consider the generator to be
     *                       converged
     * @param max_iters The maximum number of iterations of the PassGenerator to run
     */
    static void waitForConvergence(std::shared_ptr<PassGenerator> pass_generator,
                                   double min_score_diff, int max_iters)
    {
        double curr_score = 0;
        double prev_score = 0;
        for (int i = 0; i < max_iters; i++)
        {
            prev_score = curr_score;

            auto curr_pass_and_score = pass_generator->getBestPassSoFar();
            curr_score               = curr_pass_and_score.rating;

            // Run until the pass has converged with sufficient tolerance or the given
            // time has expired, whichever comes first. We also check that the score
            // is not small, otherwise we can get "false convergence" as the
            // pass just starts to "move" towards the converged point
            if (std::abs(curr_score - prev_score) < min_score_diff)
            {
                break;
            }
        }
        EXPECT_LE(std::abs(curr_score - prev_score), min_score_diff);
    }

    World world = ::TestUtil::createBlankTestingWorld();
    std::shared_ptr<PassGenerator> pass_generator;
    std::shared_ptr<const PassingConfig> passing_config;
};

TEST_F(PassGeneratorTest, check_pass_converges)
{
    // Test that we can converge to a stable pass in a scenario where there is a
    // fairly clear best pass.
    // It is difficult to update all the timestamps in the world that the pass generator
    // could use, so we don't, and hence this test does not really test convergence
    // of pass start time.

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

    pass_generator->setWorld(world);
    pass_generator->setPasserPoint(world.ball().position());

    waitForConvergence(pass_generator, 0.0015, 30);

    // Find what pass we converged to
    auto [converged_pass, converged_score] = pass_generator->getBestPassSoFar();

    // Check that we keep converging to the same pass
    for (int i = 0; i < 7; i++)
    {
        auto [pass, score] = pass_generator->getBestPassSoFar();

        EXPECT_EQ(pass.passerPoint(), converged_pass.passerPoint());
        EXPECT_LE((converged_pass.receiverPoint() - pass.receiverPoint()).length(), 0.3);
        EXPECT_LE(abs(converged_pass.speed() - pass.speed()), 0.3);
        EXPECT_LE(abs((converged_pass.startTime() - pass.startTime()).toSeconds()), 0.2);
        UNUSED(score);
    }
    UNUSED(converged_score);
}

TEST_F(PassGeneratorTest, check_passer_robot_is_ignored_for_friendly_capability)
{
    // Test that the pass generator does not converge to use the robot set as the passer

    world.updateBall(Ball(BallState({2, 0.5}, {0, 0}), Timestamp::fromSeconds(0)));
    pass_generator->setPasserPoint({2, 0.5});

    Team friendly_team(Duration::fromSeconds(10));

    // This would be the ideal robot to pass to
    Robot robot_0 = Robot(0, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                          Timestamp::fromSeconds(0));
    // This is a reasonable robot to pass to, but not the ideal
    Robot robot_1 = Robot(1, {2, -1}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                          Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1});
    world.updateFriendlyTeamState(friendly_team);
    Team enemy_team(Duration::fromSeconds(10));
    // We put a few enemies in to force the pass generator to make a decision,
    // otherwise most of the field would be a valid point to pass to
    enemy_team.updateRobots({
        Robot(0, {2, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {-2, -2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateEnemyTeamState(enemy_team);

    pass_generator->setWorld(world);
    pass_generator->setPasserRobotId(0);

    // Wait until the pass stops improving or 30 seconds, whichever comes first
    waitForConvergence(pass_generator, 0.01, 30);

    // Find what pass we converged to
    auto converged_pass_and_score          = pass_generator->getBestPassSoFar();
    auto [converged_pass, converged_score] = converged_pass_and_score;

    // We expect to have converged to a point near robot 1. The tolerance is fairly
    // generous here because the enemies on the field can "force" the point slightly
    // away from the chosen receiver robot
    EXPECT_LE((converged_pass.receiverPoint() - robot_1.position()).length(), 0.6);
    UNUSED(converged_score);
}

TEST_F(PassGeneratorTest, check_pass_does_not_converge_to_self_pass)
{
    // Test that we do not converge to a pass from the passer robot to itself

    world.updateBall(Ball(BallState({3.5, 0}, {0, 0}), Timestamp::fromSeconds(0)));
    pass_generator->setPasserPoint({3.5, 0});

    // The passer robot
    Robot passer = Robot(0, {3.7, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));

    // The potential receiver robot. Not in a great position, but the only friendly on
    // the field
    Robot receiver = Robot(1, {3.7, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                           Timestamp::fromSeconds(0));

    Team friendly_team({passer, receiver}, Duration::fromSeconds(10));
    world.updateFriendlyTeamState(friendly_team);

    pass_generator->setPasserRobotId(passer.id());

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

    pass_generator->setWorld(world);
    pass_generator->setPasserRobotId(0);

    // Wait until the pass stops improving or 30 seconds, whichever comes first
    waitForConvergence(pass_generator, 0.01, 30);

    // Find what pass we converged to
    auto converged_pass_and_score          = pass_generator->getBestPassSoFar();
    auto [converged_pass, converged_score] = converged_pass_and_score;

    ::ratePass(world, converged_pass, std::nullopt, 0, PassType::ONE_TOUCH_SHOT,
               passing_config);

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

    pass_generator->setWorld(world);

    // Set the passer point so that the only reasonable pass is to the robot
    // on the +y side
    pass_generator->setPasserPoint({3, 1});

    // Wait for the pass to converge, or 30 seconds, whichever come first
    waitForConvergence(pass_generator, 0.001, 30);

    // Find what pass we converged to
    auto converged_pass_and_score = pass_generator->getBestPassSoFar();
    auto converged_pass           = converged_pass_and_score.pass;

    // We expect to have converged to a point near the robot in +y. The tolerance is
    // fairly generous here because the enemies on the field can "force" the point
    // slightly away from the chosen receiver robot
    EXPECT_LE((converged_pass.receiverPoint() - pos_y_friendly.position()).length(), 0.7);

    // Set the passer point so that the only reasonable pass is to the robot
    // on the -y side
    pass_generator->setPasserPoint({3, -1});

    // Wait for the pass to converge, or 30 seconds, whichever come first
    waitForConvergence(pass_generator, 0.001, 30);

    // Find what pass we converged to
    converged_pass_and_score = pass_generator->getBestPassSoFar();
    converged_pass           = converged_pass_and_score.pass;

    // We expect to have converged to a point near the robot in +y. The tolerance is
    // fairly generous here because the enemies on the field can "force" the point
    // slightly away from the chosen receiver robot
    EXPECT_LE((converged_pass.receiverPoint() - neg_y_friendly.position()).length(), 0.7);
}

TEST_F(PassGeneratorTest, test_receiver_point_converges_to_point_in_target_region)
{
    // Test that when given a target region, the pass generator returns a pass
    // with the receiver point in that target region

    pass_generator->setPasserPoint({3, 3});
    Rectangle target_region({0.5, 0.5}, {1.5, -0.5});
    pass_generator->setTargetRegion(target_region);

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {1, -1.5}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, {0, 3}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateEnemyTeamState(enemy_team);
    pass_generator->setWorld(world);

    // Wait for the pass to converge, or 30 seconds, whichever come first
    waitForConvergence(pass_generator, 0.001, 30);

    // With no target region set, the pass generator would like to pass more to
    // the -y side of the field (away from the enemy and closer to the friendly).
    // With a target region set, we expect the receiver point to be within the
    // target region instead.
    auto [converged_pass, score] = pass_generator->getBestPassSoFar();
    EXPECT_TRUE(contains(target_region, converged_pass.receiverPoint()));
    UNUSED(score);
}

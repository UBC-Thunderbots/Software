/**
 * This file contains unit tests for the GradientDescent class
 */

#include "ai/passing/pass_generator.h"

#include <gtest/gtest.h>
#include <string.h>

#include "ai/passing/evaluation.h"
#include "test/test_util/test_util.h"

using namespace AI::Passing;
using namespace std::chrono_literals;

class PassGeneratorTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        world = ::Test::TestUtil::createBlankTestingWorld();
        world.updateFieldGeometry(::Test::TestUtil::createSSLDivBField());
        pass_generator = std::make_shared<PassGenerator>(world, Point(0, 0));
    }

    /**
     * Wait for the pass generator to converge to a pass
     *
     * If the pass generator does not converge within `max_num_seconds`, this will
     * cause the test to fail
     *
     * @param pass_generator Modified in-place
     * @param max_score_diff The maximum absolute difference between the scores of three
     *                       consecutive optimized passes before this function returns
     * @param max_num_seconds The maximum number of seconds that the pass optimizer
     *                        can run for before this function returns
     */
    static void waitForConvergence(std::shared_ptr<PassGenerator> pass_generator,
                                   double max_score_diff, int max_num_seconds)
    {
        int seconds_so_far     = 0;
        double curr_score      = 0;
        double prev_score      = 0;
        double prev_prev_score = 0;
        do
        {
            prev_prev_score = prev_score;
            prev_score      = curr_score;

            std::this_thread::sleep_for(1s);
            seconds_so_far++;

            auto curr_pass_and_score = pass_generator->getBestPassSoFar();
            curr_score               = curr_pass_and_score.second;

            // Run until the pass has converged with sufficient tolerance or the given
            // time has expired, whichever comes first. We also check that the score
            // is not small, otherwise we can get "false convergence" as the
            // pass just starts to "move" towards the converged point
        } while ((std::abs(curr_score - prev_score) > max_score_diff ||
                  (std::abs(curr_score - prev_prev_score) > max_score_diff) ||
                  curr_score < 0.01) &&
                 seconds_so_far < max_num_seconds);

        ASSERT_LT(seconds_so_far, max_num_seconds)
            << "Pass generator did not converge after running for " << max_num_seconds
            << " seconds";
    }

    World world;
    std::shared_ptr<PassGenerator> pass_generator;
};

TEST_F(PassGeneratorTest, check_pass_converges)
{
    // Test that we can converge to a stable pass in a fairly simple scenario.
    // It is difficult to update all the timestamps in the world that the pass generator
    // could use, so we don't, and hence this test does not really test convergence
    // of pass start time.

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(3, {-1, -1}, {-0.5, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({Robot(0, {0.5, 0.5}, {-0.5, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                             Robot(1, {-2, -2}, {-0.5, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                             Robot(2, {3, -2}, {-0.5, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                             Robot(3, {-2.5, 4}, {-0.5, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0))});
    world.updateEnemyTeamState(enemy_team);

    pass_generator->setWorld(world);

    // Wait until the pass stops improving or 30 seconds, whichever comes first
    waitForConvergence(pass_generator, 0.01, 30);

    // Find what pass we converged to
    auto [converged_pass, converged_score] = pass_generator->getBestPassSoFar();

    // Check that we keep converging to the same pass
    for (int i = 0; i < 7; i++)
    {
        std::this_thread::sleep_for(0.5s);
        auto [pass, score] = pass_generator->getBestPassSoFar();

        EXPECT_EQ(pass.passerPoint(), converged_pass.passerPoint());
        EXPECT_LE((converged_pass.receiverPoint() - pass.receiverPoint()).len(), 0.3);
        EXPECT_LE(abs(converged_pass.speed() - pass.speed()), 0.3);
        EXPECT_LE(abs((converged_pass.startTime() - pass.startTime()).getSeconds()), 0.2);
    }
}

TEST_F(PassGeneratorTest, check_passer_robot_is_ignored)
{
    // Test that the pass generator does not converge to use the robot set as the passer

    world.updateBallState(Ball({2, 0.5}, {0, 0}, Timestamp::fromSeconds(0)));
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
        Robot(0, {3, 3}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(1, {-3, -3}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
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
    EXPECT_LE((converged_pass.receiverPoint() - robot_1.position()).len(), 0.5);
}

TEST_F(PassGeneratorTest, test_passer_point_changes_are_respected)
{
    // Test that changing the passer point is reflected in the optimized passes returned

    // Put a friendly robot on the +y and -y sides of the field, both on the enemy half
    Team friendly_team(Duration::fromSeconds(10));
    Robot pos_y_friendly = Robot(0, {2, 3}, {0, 0}, Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot neg_y_friendly = Robot(1, {2, -3}, {0, 0}, Angle::zero(),
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
    auto converged_pass           = converged_pass_and_score.first;

    // We expect to have converged to a point near the robot in +y. The tolerance is
    // fairly generous here because the enemies on the field can "force" the point
    // slightly away from the chosen receiver robot
    EXPECT_LE((converged_pass.receiverPoint() - pos_y_friendly.position()).len(), 0.5);

    // Set the passer point so that the only reasonable pass is to the robot
    // on the -y side
    pass_generator->setPasserPoint({3, -1});

    // Wait for the pass to converge, or 30 seconds, whichever come first
    waitForConvergence(pass_generator, 0.001, 30);

    // Find what pass we converged to
    converged_pass_and_score = pass_generator->getBestPassSoFar();
    converged_pass           = converged_pass_and_score.first;

    // We expect to have converged to a point near the robot in +y. The tolerance is
    // fairly generous here because the enemies on the field can "force" the point
    // slightly away from the chosen receiver robot
    EXPECT_LE((converged_pass.receiverPoint() - neg_y_friendly.position()).len(), 0.5);
}

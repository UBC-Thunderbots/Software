/**
 * This file contains unit tests for the GradientDescent class
 */

#include "ai/passing/pass_generator.h"

#include <gtest/gtest.h>
#include <string.h>

#include "test/test_util/test_util.h"

using namespace AI::Passing;
using namespace std::chrono_literals;

class PassGeneratorTest : public testing::Test
{
   protected:
   protected:
    virtual void SetUp()
    {
        world = ::Test::TestUtil::createBlankTestingWorld();
        world.updateFieldGeometry(::Test::TestUtil::createSSLDivBField());
        pass_generator = std::make_shared<PassGenerator>(world, Point(1, 0));
    }

    World world;
    std::shared_ptr<PassGenerator> pass_generator;
};

TEST_F(PassGeneratorTest, check_pass_converges)
{
    // Test that we can converge to a stable pass in a fairly simple scenario

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(3, {-1, -1}, {-0.5, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world.updateFriendlyTeamState(friendly_team);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
                                    Robot(0, {1, 3.7}, {-0.5, 0}, Angle::zero(),
                                          AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                                    Robot(1, {-2, 4.0}, {-0.5, 0}, Angle::zero(),
                                    AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                                    Robot(2, {3, -2}, {-0.5, 0}, Angle::zero(),
                                    AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(3, {-2.5, 4}, {-0.5, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0))
    });
    world.updateEnemyTeamState(enemy_team);

    pass_generator->setWorld(world);

    // Wait until the pass stops improving or 30 seconds, whichever comes first
    int seconds_so_far = 0;
    double curr_score = 0;
    double prev_score = 0;
    do {
        prev_score = curr_score;
        std::this_thread::sleep_for(1s);
        seconds_so_far++;
        auto curr_pass_and_score = pass_generator->getBestPassSoFar();
        if (curr_pass_and_score){
            curr_score = curr_pass_and_score->second;
        }
    } while((abs(curr_score - prev_score) > 0.001 || curr_score < 0.2) && seconds_so_far < 30);

    ASSERT_LE(seconds_so_far, 60) << "Pass generator did not converge after running for 30 seconds";

    // Find what pass we converged to
    auto converged_pass_and_score = pass_generator->getBestPassSoFar();
    ASSERT_TRUE(converged_pass_and_score);
    auto [converged_pass, converged_score] = *converged_pass_and_score;

    // Check that we keep converging to the same pass
    for (int i = 0; i < 7; i++)
    {
        std::this_thread::sleep_for(0.5s);
        auto pass_and_score = pass_generator->getBestPassSoFar();
        ASSERT_TRUE(pass_and_score);

        auto [pass, score] = *pass_and_score;

        EXPECT_EQ(pass.passerPoint(), converged_pass.passerPoint());
        EXPECT_LE((converged_pass.receiverPoint() - pass.receiverPoint()).len(), 0.2);
        EXPECT_LE(abs(converged_pass.speed() - pass.speed()), 0.2);
        EXPECT_LE(abs((converged_pass.startTime() - pass.startTime()).getSeconds()), 0.2);
    }
}

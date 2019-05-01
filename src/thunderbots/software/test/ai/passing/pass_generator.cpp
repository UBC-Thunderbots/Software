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
                             Robot(4, {0.5, 4}, {-0.5, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0))});
    world.updateEnemyTeamState(enemy_team);

    pass_generator->setWorld(world);


    std::this_thread::sleep_for(15s);

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
        EXPECT_LE(
            (converged_pass.receiverPoint() - pass.receiverPoint()).len(),
            0.2);
        EXPECT_LE(abs(converged_pass.speed() - pass.speed()), 0.2);
        EXPECT_LE(abs((converged_pass.startTime() - pass.startTime())
                          .getSeconds()),
                  0.2);
    }
}

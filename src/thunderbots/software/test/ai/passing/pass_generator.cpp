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
        pass_generator = std::make_shared<PassGenerator>(0.05, world, Point(1, 0));
    }

    World world;
    std::shared_ptr<PassGenerator> pass_generator;
};

TEST_F(PassGeneratorTest, check_pass_converges)
{
    // Test that the pass converges to a stable pass when there is a somewhat random
    // scattering of friendly and enemy robots around the field
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots(
        {Robot(1, {2, 2}, {-0.5, 0}, Angle::zero(), AngularVelocity::zero(),
               Timestamp::fromSeconds(0)),
         Robot(2, {-3, 1}, {-0.5, 0}, Angle::zero(), AngularVelocity::zero(),
               Timestamp::fromSeconds(0)),
         Robot(3, {-1, -1}, {-0.5, 0}, Angle::zero(), AngularVelocity::zero(),
               Timestamp::fromSeconds(0)),
         Robot(4, {0.2, 0.5}, {-0.5, 0}, Angle::zero(), AngularVelocity::zero(),
               Timestamp::fromSeconds(0))});
    world.updateFriendlyTeamState(friendly_team);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({Robot(0, {1, 0}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                             Robot(1, {2, 1}, {-0.5, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                             Robot(2, {3, 2}, {-0.5, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                             Robot(3, {4, -1}, {-0.5, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                             Robot(4, {0.5, 4}, {-0.5, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0))});
    world.updateEnemyTeamState(enemy_team);

    pass_generator->setWorld(world);

    std::this_thread::sleep_for(10s);

    // Find what pass we converged to
    auto converged_pass_opt = pass_generator->getBestPassSoFar();
    ASSERT_TRUE(converged_pass_opt);

    // Check that we keep converging to the same pass
    for (int i = 0; i < 7; i++)
    {
        std::this_thread::sleep_for(0.5s);
        auto new_pass_opt = pass_generator->getBestPassSoFar();
        ASSERT_TRUE(new_pass_opt);

        EXPECT_EQ(converged_pass_opt->passerPoint(), new_pass_opt->passerPoint());
        EXPECT_LE(
            (converged_pass_opt->receiverPoint() - new_pass_opt->receiverPoint()).len(),
            0.2);
        EXPECT_LE(converged_pass_opt->speed() - new_pass_opt->speed(), 0.2);
        EXPECT_LE(abs((converged_pass_opt->startTime() - new_pass_opt->startTime())
                          .getSeconds()),
                  0.2);
    }
}

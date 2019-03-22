/**
 * This file contains unit tests for the GradientDescent class
 */

#include "ai/passing/pass_generator.h"

#include <gtest/gtest.h>
#include <string.h>

#include "test/test_util/test_util.h"

using namespace AI::Passing;

class PassGeneratorTest : public testing::Test
{
   protected:
   protected:
    virtual void SetUp()
    {
        world = ::Test::TestUtil::createBlankTestingWorld();
        Team friendly_team(Duration::fromSeconds(10));
        friendly_team.updateRobots(
            {Robot(0, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                   Timestamp::fromSeconds(0))});
        world.updateFriendlyTeamState(friendly_team);
        world.updateFieldGeometry(::Test::TestUtil::createSSLDivBField());
        pass_generator = std::make_shared<PassGenerator>(0.0);
        pass_generator->setWorld(world);
    }

    World world;
    std::shared_ptr<PassGenerator> pass_generator;
};

TEST_F(PassGeneratorTest, static_convergence_towards_target_region)
{
    // Test that given enough time and a static world with no robots, we converge to a
    // pass near the enemy team goal

    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::optional<Pass> pass1 = pass_generator->getBestPassSoFar();

    // Make sure we got some pass
    ASSERT_TRUE(pass1);

    // Check that the pass receiver point is approximately the one we expect
    EXPECT_GE(pass1->receiverPoint().x(), 3.2);
    EXPECT_LE(pass1->receiverPoint().x(), 4.5);
    EXPECT_NEAR(pass1->receiverPoint().y(), 0, 0.01);
}

/**
 * This file contains unit tests for the GradientDescent class
 */

#include "ai/passing/pass_generator.h"

#include <gtest/gtest.h>
#include <string.h>

#include "test/test_util/test_util.h"

using namespace AI::Passing;

// TODO: performance tests? That's gonna be tricky........

class PassGeneratorTest : public testing::Test
{
   protected:
   protected:
    virtual void SetUp()
    {
        world = ::Test::TestUtil::createBlankTestingWorld();
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

    // Check that the pass is across the half-line towards the enemy goal
    EXPECT_GE(pass1->receiverPoint().x(), 0.1);
    // Currently we just generate receiver points at (0,0), so y should be 0
    EXPECT_EQ(pass1->receiverPoint().y(), 0);

    // Run a bit more
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::optional<Pass> pass2 = pass_generator->getBestPassSoFar();

    // Check that we're moving towards the goal
    ASSERT_TRUE(pass2);
    EXPECT_GE(pass2->receiverPoint().x(), pass1->receiverPoint().x());

    // Check that we're aren't moving at all in y
    EXPECT_EQ(pass2->receiverPoint().y(), 0);

    // TODO (Issue #323): Check more things here when we're actually generating passes
}

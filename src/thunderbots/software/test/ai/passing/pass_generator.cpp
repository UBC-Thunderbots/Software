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


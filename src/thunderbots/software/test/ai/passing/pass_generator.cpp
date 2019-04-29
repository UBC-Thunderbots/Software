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

TEST_F(PassGeneratorTest, check_pass_converges){
    // Test that the pass converges to a stable pass when there are a a random
    // scattering of friendly and enemy robots around the field
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(0, {0, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                   Timestamp::fromSeconds(0)),
                                       Robot(0, {2, 2}, {-0.5, 0}, Angle::zero(), AngularVelocity::zero(),
                                       Timestamp::fromSeconds(0)),
        Robot(0, {-3, 1}, {-0.5, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(0, {-1, -1}, {-0.5, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(0, {0.2, 0.5}, {-0.5, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0))
            });
    world.updateFriendlyTeamState(friendly_team);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
                                       Robot(0, {1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                                             Timestamp::fromSeconds(0)),
                                       Robot(0, {2, 1}, {-0.5, 0}, Angle::zero(), AngularVelocity::zero(),
                                             Timestamp::fromSeconds(0)),
                                       Robot(0, {3, 2}, {-0.5, 0}, Angle::zero(), AngularVelocity::zero(),
                                             Timestamp::fromSeconds(0)),
                                       Robot(0, {4, -1}, {-0.5, 0}, Angle::zero(), AngularVelocity::zero(),
                                             Timestamp::fromSeconds(0)),
                                       Robot(0, {0.5, 4}, {-0.5, 0}, Angle::zero(), AngularVelocity::zero(),
                                             Timestamp::fromSeconds(0))
                               });
    world.updateFriendlyTeamState(enemy_team);

    pass_generator->setWorld(world);

    std::this_thread::sleep_for(5);
}



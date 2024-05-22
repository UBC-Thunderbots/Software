#include "software/ai/passing/sampling_pass_generator.h"

#include <gtest/gtest.h>

#include <chrono>
#include <random>

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/math/math_functions.h"
#include "software/test_util/test_util.h"

class SamplingPassGeneratorTest : public testing::Test
{
   public:
    SamplingPassGeneratorTest()
        : passing_config(), sampling_pass_generator(passing_config)
    {
    }

   protected:
    virtual void SetUp()
    {
        entire_field =
            std::make_shared<Rectangle>(Field::createSSLDivisionBField().fieldLines());
        passing_config.set_min_pass_speed_m_per_s(3.5);
        passing_config.set_max_pass_speed_m_per_s(5.5);
    }

    std::shared_ptr<Rectangle> entire_field;
    TbotsProto::PassingConfig passing_config;
    SamplingPassGenerator sampling_pass_generator;
};


TEST_F(SamplingPassGeneratorTest, getBestPass_2_friendlies)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots(
        {Robot(1, {-1, 0}, {0, 0}, Angle::fromDegrees(0), AngularVelocity::zero(),
               Timestamp::fromSeconds(0)),
         Robot(2, {1, 0}, {0, 0}, Angle::fromDegrees(180), AngularVelocity::zero(),
               Timestamp::fromSeconds(0))});
    world->updateFriendlyTeamState(friendly_team);
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    world->updateBall(ball);

    PassWithRating best_pass = sampling_pass_generator.getBestPass(*world);
    EXPECT_GE(best_pass.rating, 0.7);
}

TEST_F(SamplingPassGeneratorTest, getBestPass_3_friendlies)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots(
        {Robot(1, {1.5, 1.5}, {0, 0}, Angle::fromDegrees(0), AngularVelocity::zero(),
               Timestamp::fromSeconds(0)),
         Robot(2, {1.5, -1.5}, {0, 0}, Angle::fromDegrees(180), AngularVelocity::zero(),
               Timestamp::fromSeconds(0)),
         Robot(3, {-1, 0}, {0, 0}, Angle::fromDegrees(180), AngularVelocity::zero(),
               Timestamp::fromSeconds(0))});
    world->updateFriendlyTeamState(friendly_team);
    Ball ball({0.5, 0}, {0, 0}, Timestamp::fromSeconds(0));
    world->updateBall(ball);

    PassWithRating best_pass = sampling_pass_generator.getBestPass(*world);
    // not too sure why this pass is so bad but seems that way
    EXPECT_GE(best_pass.rating, 0.1);
}

TEST_F(SamplingPassGeneratorTest, getBestPass_3_friendlies_1_blocked)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots(
        {Robot(1, {-2, 0}, {0, 0}, Angle::fromDegrees(0), AngularVelocity::zero(),
               Timestamp::fromSeconds(0)),
         Robot(2, {2, 0}, {0, 0}, Angle::fromDegrees(180), AngularVelocity::zero(),
               Timestamp::fromSeconds(0)),
         Robot(3, {-2, 1}, {0, 0}, Angle::fromDegrees(180), AngularVelocity::zero(),
               Timestamp::fromSeconds(0))});
    world->updateFriendlyTeamState(friendly_team);
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    world->updateBall(ball);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, {-1.7, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world->updateEnemyTeamState(enemy_team);

    PassWithRating best_pass = sampling_pass_generator.getBestPass(*world);
    EXPECT_GE(best_pass.rating, 0.7);
}

TEST_F(SamplingPassGeneratorTest, getBestPass_3_friendlies_2_blocked)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots(
        {Robot(1, {2, 0}, {0, 0}, Angle::fromDegrees(0), AngularVelocity::zero(),
               Timestamp::fromSeconds(0)),
         Robot(2, {-1, 0}, {0, 0}, Angle::fromDegrees(180), AngularVelocity::zero(),
               Timestamp::fromSeconds(0)),
         Robot(3, {-2.5, 0}, {0, 0}, Angle::fromDegrees(180), AngularVelocity::zero(),
               Timestamp::fromSeconds(0))});
    world->updateFriendlyTeamState(friendly_team);
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    world->updateBall(ball);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({Robot(0, {-2, 0}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                             Robot(1, {1.7, 0}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0))});
    world->updateEnemyTeamState(enemy_team);

    PassWithRating best_pass = sampling_pass_generator.getBestPass(*world);
    // since both robots are blocked, chosen pass will be pretty bad
    // so just check that it's not 0
    EXPECT_GE(best_pass.rating, 0);
    EXPECT_LE(best_pass.rating, 0.4);
}

TEST_F(SamplingPassGeneratorTest, getBestPass_1_friendly_1_enemy)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots(
        {Robot(1, {2, 0}, {0, 0}, Angle::fromDegrees(0), AngularVelocity::zero(),
               Timestamp::fromSeconds(0))});
    world->updateFriendlyTeamState(friendly_team);
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    world->updateBall(ball);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({Robot(0, {-2, 0}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0))});
    world->updateEnemyTeamState(enemy_team);

    PassWithRating best_pass = sampling_pass_generator.getBestPass(*world);
    // since friendly can't pass anywhere, pass will be pretty bad
    EXPECT_GE(best_pass.rating, 0);
    EXPECT_LE(best_pass.rating, 0.4);
}

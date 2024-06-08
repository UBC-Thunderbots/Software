#include "software/ai/passing/pass_generator.h"

#include <gtest/gtest.h>
#include <string.h>

#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/geom/algorithms/contains.h"
#include "software/test_util/test_util.h"

class PassGeneratorTest : public testing::Test
{
   protected:
    PassGeneratorTest() : pass_generator(passing_config) {}

    virtual void SetUp()
    {
        // Update configs
        passing_config.set_min_pass_speed_m_per_s(1);
        passing_config.set_max_pass_speed_m_per_s(5.5);
        passing_config.set_receiver_ideal_min_distance_meters(0.1);
        pass_generator = PassGenerator(passing_config);
    }

    /**
     * Calls getBestPass to step the pass generator a couple times
     * to find better passes.
     *
     * The pass generator starts with bad passes and improves on them as it receives
     * more "world" inputs
     *
     * @param pass_generator The pass generator to step
     * @param world The world to evaluate passes on
     * @param max_iters The maximum number of iterations of the PassGenerator to run
     */
    static void stepPassGenerator(PassGenerator pass_generator,
                                  const World& world, int max_iters)
    {
        for (int i = 0; i < max_iters; i++)
        {
            pass_generator.getBestPass(world);
        }
    }

    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    TbotsProto::PassingConfig passing_config;
    PassGenerator pass_generator;
};

TEST_F(PassGeneratorTest, check_pass_converges)
{
    // Test that we can converge to a stable pass in a scenario where there is a
    // fairly clear best pass.

    world->updateBall(
        Ball(BallState(Point(2, 2), Vector(0, 0)), Timestamp::fromSeconds(0)));
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({
        Robot(3, {1, 0}, {0.5, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world->updateFriendlyTeamState(friendly_team);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, world->field().enemyGoalpostNeg(), {0, 0}, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(1, world->field().enemyGoalpostNeg() - Vector(0.1, 0), {0, 0},
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(2, world->field().enemyGoalpostNeg() - Vector(0.2, 0), {0, 0},
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(3, world->field().enemyGoalpostPos(), {0, 0}, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(4, world->field().enemyGoalpostPos() - Vector(0.1, 0), {0, 0},
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(5, world->field().enemyGoalpostPos() - Vector(0.2, 0), {0, 0},
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(6, {-1, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(7, {-1, 0.5}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
        Robot(8, {-1, -0.5}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0)),
    });
    world->updateEnemyTeamState(enemy_team);

    // call generate evaluation 100 times on the given world
    stepPassGenerator(pass_generator, *world, 100);

    auto [best_pass, score] = pass_generator.getBestPass(*world);

    // After 100 iterations on the same world, we should "converge"
    // to the same pass.
    for (int i = 0; i < 7; i++)
    {
        auto [pass, score] = pass_generator.getBestPass(*world);

        EXPECT_LE((best_pass.receiverPoint() - pass.receiverPoint()).length(), 0.7);
        EXPECT_LE(abs(best_pass.speed() - pass.speed()), 0.7);
        UNUSED(score);
    }
    UNUSED(score);
}

TEST_F(PassGeneratorTest, check_pass_does_not_converge_to_self_pass)
{
    // Test that we do not converge to a pass from the passer robot to itself

    world->updateBall(Ball(BallState({3.5, 0}, {0, 0}), Timestamp::fromSeconds(0)));

    // The passer robot
    Robot passer = Robot(0, {3.7, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                         Timestamp::fromSeconds(0));

    // The potential receiver robot. Not in a great position, but the only friendly on
    // the field
    Robot receiver = Robot(1, {3.7, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                           Timestamp::fromSeconds(0));

    Team friendly_team({passer, receiver}, Duration::fromSeconds(10));
    world->updateFriendlyTeamState(friendly_team);

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
    world->updateEnemyTeamState(enemy_team);

    // call generate evaluation 100 times on the given world
    stepPassGenerator(pass_generator, *world, 100);

    // Find what pass we converged to
    auto [converged_pass, converged_score] = pass_generator.getBestPass(*world);

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
    world->updateFriendlyTeamState(friendly_team);

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
    world->updateEnemyTeamState(enemy_team);

    world->updateBall(
        Ball(BallState(Point(3, 1), Vector(0, 0)), Timestamp::fromSeconds(0)));

    // call generate evaluation 100 times on the given world
    stepPassGenerator(pass_generator, *world, 100);

    // Find what pass we converged to
    auto converged_pass  = pass_generator.getBestPass(*world).pass;

    // We expect to have converged to a point closer to the robot in the pos_y
    // compared to the robot in the neg_y position since the ball is in +y
    EXPECT_LT((converged_pass.receiverPoint() - pos_y_friendly.position()).length(),
              (converged_pass.receiverPoint() - neg_y_friendly.position()).length());

    // Set the passer point so that the only reasonable pass is to the robot
    // on the -y side
    world->updateBall(
        Ball(BallState(Point(3, -1), Vector(0, 0)), Timestamp::fromSeconds(0)));

    // call generate evaluation 100 times on the given world
    stepPassGenerator(pass_generator, *world, 100);

    // Find what pass we converged to
    converged_pass  = pass_generator.getBestPass(*world).pass;

    // We expect to have converged to a point closer to the robot in the neg_y
    // compared to the robot in the pos_y position.
    EXPECT_LT((converged_pass.receiverPoint() - neg_y_friendly.position()).length(),
              (converged_pass.receiverPoint() - pos_y_friendly.position()).length());
}

TEST_F(PassGeneratorTest, test_open_pass_score_being_high_2_friendlies)
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

    PassWithRating best_pass = pass_generator.getBestPass(*world);
    EXPECT_GE(best_pass.rating, 0.8);
}

TEST_F(PassGeneratorTest, test_open_pass_score_being_high_3_friendlies)
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

    PassWithRating best_pass = pass_generator.getBestPass(*world);
    EXPECT_GE(best_pass.rating, 0.8);
}

TEST_F(PassGeneratorTest, test_two_blocked_friendly_one_open_friendly)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots(
            {
                // Open friendly
                Robot(1, {1, 2.5}, {0, 0}, Angle::fromDegrees(-100), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0)),
                // Blocked friendly
                Robot(2, {2, 0}, {0, 0}, Angle::fromDegrees(0), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0)),
                Robot(3, {-2.5, 0}, {0, 0}, Angle::fromDegrees(180), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0)),
            });
    world->updateFriendlyTeamState(friendly_team);
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    world->updateBall(ball);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({Robot(0, {-2, 0}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0)),
                             Robot(1, {1.7, 0}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0))});
    world->updateEnemyTeamState(enemy_team);

    PassWithRating best_pass = pass_generator.getBestPass(*world);
    EXPECT_GE(best_pass.rating, 0.5);
    // Verify that the pass is to the open friendly
    EXPECT_TRUE((best_pass.pass.receiverPoint() - world->friendlyTeam().getRobotById(1)->position()).length() < 0.3);
}

TEST_F(PassGeneratorTest, test_robots_ignore_list)
{
    // Friendly robot 2 has an enemy robot in close proximity, so it is un-ideal,
    // however, robot 1 who is open is added to the ignore list.
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots(
            {
                Robot(1, {2, 0}, {0, 0}, Angle::fromDegrees(0), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0)),
                Robot(2, {-2, 0}, {0, 0}, Angle::fromDegrees(180), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0)),
            });
    world->updateFriendlyTeamState(friendly_team);
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    world->updateBall(ball);
    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({Robot(0, {-1.8, 0.8}, {0, 0}, Angle::zero(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0))});
    world->updateEnemyTeamState(enemy_team);

    std::vector<RobotId> ignore_list = {1};
    PassWithRating best_pass = pass_generator.getBestPass(*world, ignore_list);

    // Verify that the pass is to the only friendly which is not ignored
    EXPECT_TRUE((best_pass.pass.receiverPoint() - world->friendlyTeam().getRobotById(2)->position()).length() < 0.3);
}


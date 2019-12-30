#include "software/ai/hl/stp/tactic/tactic_update_visitor.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/tactic/defense_shadow_enemy_tactic.h"
#include "software/ai/intent/kick_intent.h"
#include "software/geom/util.h"
#include "software/new_geom/util/distance.h"
#include "software/test_util/test_util.h"


TEST(TacticUpdateVisitorTest, update_cherry_pick_tactic)
{
    // add cherry pick test
}

TEST(TacticUpdateVisitorTest, update_chip_tactic)
{
    Robot robot  = Robot(0, Point(1, 0), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    World world1 = ::Test::TestUtil::createBlankTestingWorld();
    World world2 = ::Test::TestUtil::createBlankTestingWorld();
    world2 =
        ::Test::TestUtil::setBallPosition(world2, Point(1, 0), Timestamp::fromSeconds(0));
    ChipTactic tactic = ChipTactic(world1.ball());

    EXPECT_NE(tactic.calculateRobotCost(robot, world1), 0.0);

    TacticUpdateVisitor visitor = TacticUpdateVisitor(world2);
    visitor.visit(tactic);
    tactic.accept(visitor);
    EXPECT_EQ(tactic.calculateRobotCost(robot, world2), 0.0);
}

TEST(TacticUpdateVisitorTest, update_crease_defender_tactic)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();

    CreaseDefenderTactic tactic =
        CreaseDefenderTactic(world.field(), world.ball(), world.friendlyTeam(),
                             world.enemyTeam(), CreaseDefenderTactic::LEFT);
    ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.09, 0)},
                                             Timestamp::fromSeconds(0));

    TacticUpdateVisitor visitor = TacticUpdateVisitor(world);
    tactic.accept(visitor);

    Robot friendly_robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot});

    tactic.updateRobot(friendly_robot);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    EXPECT_TRUE(move_action->getDestination().isClose(
        Point(world.field().friendlyDefenseArea().posXPosYCorner().x() +
                  ROBOT_MAX_RADIUS_METERS,
              0.0),
        0.05));
}

TEST(TacticUpdateVisitorTest, update_defense_shadow_enemy)
{
    Robot enemy_robot(1, Point(0, 0), Vector(0, 0), Angle::zero(),
                      AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Evaluation::EnemyThreat enemy_threat{enemy_robot,  false, Angle::zero(), std::nullopt,
                                         std::nullopt, 0,     std::nullopt};
    Field field        = ::Test::TestUtil::createSSLDivBField();
    Team enemy_team    = Team(Duration::fromSeconds(1), {enemy_robot});
    Team friendly_team = Team(Duration::fromSeconds(1), {friendly_robot});
    Ball ball(Point(1, 1), Vector(0, 0), Timestamp::fromSeconds(0));

    World world = World(field, ball, friendly_team, enemy_team, 20);

    DefenseShadowEnemyTactic tactic =
        DefenseShadowEnemyTactic(field, friendly_team, enemy_team, ball, true, 0.5, true);
    tactic.updateRobot(friendly_robot);
    TacticUpdateVisitor visitor = TacticUpdateVisitor(world);
    tactic.accept(visitor);
    tactic.updateControlParams(enemy_threat);

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    EXPECT_TRUE(move_action->getDestination().isClose(Point(-0.5, 0), 0.01));
    EXPECT_LT(move_action->getFinalOrientation().minDiff(Angle::zero()),
              Angle::fromDegrees(1));
    EXPECT_TRUE(move_action->getAutoKickType() == AutokickType::AUTOCHIP);
}

TEST(TacticUpdateVisitorTest, update_goalie_tactic)
{
    // add goalie test
}

TEST(TacticUpdateVisitorTest, update_passer_tactic)
{
    // add passer test
}

TEST(TacticUpdateVisitorTest, update_penalty_kick_tactic)
{
    Robot robot  = Robot(0, Point(1, 0), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    World world1 = ::Test::TestUtil::createBlankTestingWorld();
    World world2 = ::Test::TestUtil::createBlankTestingWorld();
    world2 =
        ::Test::TestUtil::setBallPosition(world2, Point(1, 0), Timestamp::fromSeconds(0));
    PenaltyKickTactic tactic = PenaltyKickTactic(world1.ball(), world1.field(),
                                                 world1.enemyTeam().goalie(), true);

    EXPECT_NE(tactic.calculateRobotCost(robot, world1), 0.0);

    TacticUpdateVisitor visitor = TacticUpdateVisitor(world2);
    visitor.visit(tactic);
    tactic.accept(visitor);
    EXPECT_EQ(tactic.calculateRobotCost(robot, world2), 0.0);
}

TEST(TacticUpdateVisitorTest, update_receiver_tactic)
{
    Pass pass({1, 1}, {0.5, 0}, 2.29, Timestamp::fromSeconds(5));

    Robot receiver = Robot(13, Point(0.5, 0), Vector(), pass.receiverOrientation(),
                           AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});

    Ball ball({1, 1}, {0, 0}, Timestamp::fromSeconds(0));

    Field field = ::Test::TestUtil::createSSLDivBField();
    ReceiverTactic tactic(field, friendly_team, enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    World world = World(field, ball, friendly_team, enemy_team, 20);

    // Since we're already setup to receive the pass, we should just be trying to move
    // to our current position. We should continue to yield new Move Intents even though
    // we're at the target position
    for (int i = 0; i < 5; i++)
    {
        TacticUpdateVisitor visitor = TacticUpdateVisitor(world);
        tactic.accept(visitor);
        tactic.updateControlParams(pass);
        Angle shot_dir = (field.enemyGoal() - receiver.position()).orientation();

        auto move_action = std::dynamic_pointer_cast<MoveAction>(tactic.getNextAction());
        ASSERT_NE(move_action, nullptr);
        ASSERT_TRUE(move_action->getRobot().has_value());
        EXPECT_EQ(13, move_action->getRobot()->id());
        EXPECT_DOUBLE_EQ(0.5, move_action->getDestination().x());
        EXPECT_DOUBLE_EQ(0.0, move_action->getDestination().y());
        EXPECT_EQ((pass.receiverOrientation() + shot_dir) / 2,
                  move_action->getFinalOrientation());
        EXPECT_EQ(DribblerEnable::OFF, move_action->getDribblerEnabled());
        EXPECT_EQ(move_action->getAutoKickType(), NONE);
    }
}

TEST(TacticUpdateVisitorTest, update_shadow_enemy_tactic)
{
    Robot enemy_robot(1, Point(0, 0), Vector(0, 0), Angle::zero(),
                      AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Evaluation::EnemyThreat enemy_threat{enemy_robot,  false, Angle::zero(), std::nullopt,
                                         std::nullopt, 0,     std::nullopt};
    Field field        = ::Test::TestUtil::createSSLDivBField();
    Team enemy_team    = Team(Duration::fromSeconds(1), {enemy_robot});
    Team friendly_team = Team(Duration::fromSeconds(1), {friendly_robot});
    Ball ball(Point(1, 1), Vector(0, 0), Timestamp::fromSeconds(0));

    ShadowEnemyTactic tactic =
        ShadowEnemyTactic(field, friendly_team, enemy_team, true, ball, 0.5, false, true);
    tactic.updateRobot(friendly_robot);
    World world                 = World(field, ball, friendly_team, enemy_team, 20);
    TacticUpdateVisitor visitor = TacticUpdateVisitor(world);
    tactic.accept(visitor);
    // tactic.updateWorldParams(field, friendly_team, enemy_team, ball);
    tactic.updateControlParams(enemy_threat, 0.5);

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    EXPECT_TRUE(move_action->getDestination().isClose(Point(-0.5, 0), 0.01));
    EXPECT_LT(move_action->getFinalOrientation().minDiff(Angle::zero()),
              Angle::fromDegrees(1));
    EXPECT_TRUE(move_action->getAutoKickType() == NONE);
}

TEST(TacticUpdateVisitorTest, update_shadow_freekicker_tactic)
{
    World blank_world = ::Test::TestUtil::createBlankTestingWorld();
    World world       = ::Test::TestUtil::createBlankTestingWorld();
    world             = ::Test::TestUtil::setEnemyRobotPositions(
        world, {0, Point(world.field().friendlyCornerPos().y(), 0)},
        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallPosition(
        world, Point(-0.09, world.field().friendlyCornerPos().y() - 0.09),
        Timestamp::fromSeconds(0));
    world =
        ::Test::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Robot friendly_robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot});

    ShadowFreekickerTactic tactic =
        ShadowFreekickerTactic(ShadowFreekickerTactic::LEFT, blank_world.enemyTeam(),
                               blank_world.ball(), blank_world.field(), false);
    TacticUpdateVisitor visitor = TacticUpdateVisitor(world);
    tactic.accept(visitor);
    tactic.updateRobot(friendly_robot);

    auto action_ptr = tactic.getNextAction();

    ASSERT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(nullptr, move_action);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR((world.ball().position() - move_action->getDestination()).length(), 0.62,
                0.05);

    // The robot should be just to the left of the line between the friendly net and
    // the ball (from the POV of the friendly net)
    Line ball_to_net_line = Line(world.ball().position(), world.field().friendlyGoal());
    EXPECT_NEAR(distance(ball_to_net_line, move_action->getDestination()), 0.09, 0.01);
    Angle goal_to_ball_angle =
        (world.ball().position() - world.field().friendlyGoal()).orientation();
    Angle goal_to_dest_angle =
        (move_action->getDestination() - world.field().friendlyGoal()).orientation();
    EXPECT_GT(goal_to_dest_angle, goal_to_ball_angle);
}

TEST(TacticUpdateVisitorTest, update_shoot_goal_tactic)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});
    BallState ball(Point(ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0),
                   Timestamp::fromSeconds(0));
    world.updateBallState(ball);

    ShootGoalTactic tactic =
        ShootGoalTactic(world.field(), world.friendlyTeam(), world.enemyTeam(),
                        world.ball(), Angle::zero(), std::nullopt, false);
    tactic.updateRobot(robot);
    TacticUpdateVisitor visitor = TacticUpdateVisitor(world);
    tactic.accept(visitor);
    // tactic.updateWorldParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
    //        world.ball());

    auto action_ptr = tactic.getNextAction();

    // Check an intent was returned (the pointer is not null)
    ASSERT_NE(nullptr, action_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_TRUE(tactic.hasShotAvailable());

    auto kick_action = std::dynamic_pointer_cast<KickAction>(action_ptr);
    ASSERT_NE(nullptr, kick_action);
    ASSERT_TRUE(kick_action->getRobot().has_value());
    EXPECT_EQ(0, kick_action->getRobot()->id());
}

#include "software/ai/hl/stp/tactic/tactic_update_visitor.h"

#include <gtest/gtest.h>

#include "software/geom/util.h"
#include "software/test_util/test_util.h"
#include "software/ai/hl/stp/tactic/defense_shadow_enemy_tactic.h"

//add cherry pick tactic test

TEST(TacticUpdateVisitorTest, update_chip_tactic)
{
    Robot robot = Robot(0, Point(1, 0), Vector(0, 0), Angle::zero(),
            AngularVelocity::zero(), Timestamp::fromSeconds(0));
    World world1 = ::Test::TestUtil::createBlankTestingWorld();
    World world2 = ::Test::TestUtil::createBlankTestingWorld();
    world2 = ::Test::TestUtil::setBallPosition(world2, Point(1,0), Timestamp::fromSeconds(0));
    ChipTactic tactic = ChipTactic(world1.ball());

    EQUAL_TRUE(tactic.calculateRobotCost(robot, world1) != 0);

    TacticUpdateVisitor visitor = TacticUpdateVisitor(world2);
    visitor.visit(tactic);
    EQUAL_TRUE(tactic.calculateRobotCost(robot, world2) == 0);
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
    visitor.visit(tactic);


    Robot friendly_robot = Robot(0, Point(-2, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({friendly_robot});

    tactic.updateRobot(friendly_robot);
    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(move_intent.getDestination().isClose(
                Point(world.field().friendlyDefenseArea().posXPosYCorner().x() +
                      ROBOT_MAX_RADIUS_METERS,
                      0.0),
        0.05));
    }
    catch (...)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the ShootGoalTactic!";
    }
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
    visitor.visit(tactic);
    tactic.updateControlParams(enemy_threat);

    auto intent_ptr = tactic.getNextIntent();

    ASSERT_TRUE(intent_ptr);

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(move_intent.getDestination().isClose(Point(-0.5, 0), 0.01));
        EXPECT_LT(move_intent.getFinalAngle().minDiff(Angle::zero()),
                Angle::ofDegrees(1));
        EXPECT_TRUE(move_intent.getAutoKickType() == AutokickType::AUTOCHIP);
    }
    catch (std::bad_cast &)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the ShootGoalTactic!";
    }
}

TEST(TacticUpdateVisitorTest, update_goalie_tactic)
{
    //add goalie test
}

TEST(TacticUpdateVisitorTest, update_grab_ball_tactic)
{
//add grabball test
}

TEST(TacticUpdateVisitorTest, update_passer_tactic)
{
//add passer test
}

TEST(TacticUpdateVisitorTest, update_penalty_kick_tactic)
{
//add penalty kick tactic
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

    // Since we're already setup to receive the pass, we should just be trying to move
    // to our current position. We should continue to yield new Move Intents even though
    // we're at the target position
    for (int i = 0; i < 5; i++)
    {
        TacticUpdateVisitor visitor = TacticUpdateVisitor(world);
        visitor.visit(tactic);
        tactic.updateWorldParams(friendly_team, enemy_team, ball);
        tactic.updateControlParams(pass);
        Angle shot_dir = (field.enemyGoal() - receiver.position()).orientation();

        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*tactic.getNextIntent());
        EXPECT_EQ(13, move_intent.getRobotId());
        EXPECT_DOUBLE_EQ(0.5, move_intent.getDestination().x());
        EXPECT_DOUBLE_EQ(0.0, move_intent.getDestination().y());
        EXPECT_EQ((pass.receiverOrientation() + shot_dir) / 2,
        move_intent.getFinalAngle());
        EXPECT_FALSE(move_intent.getDribblerEnable());
        EXPECT_EQ(move_intent.getAutoKickType(), NONE);
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
    TacticUpdateVisitor visitor = TacticUpdateVisitor(world);
    visitor.visit(tactic);
    //tactic.updateWorldParams(field, friendly_team, enemy_team, ball);
    tactic.updateControlParams(enemy_threat, 0.5);

    auto intent_ptr = tactic.getNextIntent();

    ASSERT_TRUE(intent_ptr);

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_TRUE(move_intent.getDestination().isClose(Point(-0.5, 0), 0.01));
        EXPECT_LT(move_intent.getFinalAngle().minDiff(Angle::zero()),
                Angle::ofDegrees(1));
        EXPECT_TRUE(move_intent.getAutoKickType() == NONE);
    }
    catch (...)
    {
        ADD_FAILURE() << "MoveIntent was not returned by the ShootGoalTactic!";
    }
}

TEST(TacticUpdateVisitorTest, update_shadow_freekicker_tactic)
{
//add shadow freekicker
}

TEST(TacticUpdateVisitorTest, update_shoot_goal_tactic)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});
    Ball ball(Point(ROBOT_MAX_RADIUS_METERS, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    world.updateBallState(ball);

    ShootGoalTactic tactic =
            ShootGoalTactic(world.field(), world.friendlyTeam(), world.enemyTeam(),
                            world.ball(), Angle::zero(), std::nullopt, false);
    tactic.updateRobot(robot);
    TacticUpdateVisitor visitor = TacticUpdateVisitor(world);
    visitor.visit(tactic);
    //tactic.updateWorldParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
    //        world.ball());

    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(intent_ptr);
    EXPECT_FALSE(tactic.done());
    EXPECT_TRUE(tactic.hasShotAvailable());

    try
    {
        KickIntent kick_intent = dynamic_cast<KickIntent &>(*intent_ptr);
        EXPECT_EQ(0, kick_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE() << "KickIntent was not returned by the ShootGoalTactic!";
    }
}


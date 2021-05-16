#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_tactic_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(PenaltyKickTacticFSM, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().friendlyPenaltyMark(),
                                        Timestamp::fromSeconds(0));
    Robot robot = ::TestUtil::createRobotAtPos(world.field().friendlyPenaltyMark());

    FSM<PenaltyKickTacticFSM> fsm;

    PenaltyKickTacticFSM::ControlParams control_params{};

    fsm.process_event(PenaltyKickTacticFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));

    Point position = world.field().enemyGoalCenter() + Vector(-1, 0);
    robot          = ::TestUtil::createRobotAtPos(position);
    world = ::TestUtil::setBallPosition(world, position + Vector(ROBOT_MAX_RADIUS_METERS, 0), Timestamp::fromSeconds(1));
    fsm.process_event(PenaltyKickTacticFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<KickFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<KickFSM>)>(
        boost::sml::state<GetBehindBallFSM>));

    fsm.process_event(PenaltyKickTacticFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<KickFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<KickFSM>)>(
        boost::sml::state<KickFSM::KickState>));

    world = ::TestUtil::setBallPosition(world, world.field().enemyGoalCenter(),
                                        Timestamp::fromSeconds(2));
    world = ::TestUtil::setBallVelocity(world, Vector(5, 0), Timestamp::fromSeconds(2));
    fsm.process_event(PenaltyKickTacticFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));

    EXPECT_TRUE(fsm.is(boost::sml::X));
}

TEST(PenaltyKickTacticFSMTest, no_enemy_goalie)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, Point(4, 0),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Vector behind_ball_direction =
        -(world.ball().position() - world.field().enemyGoalCenter()).normalize();
    Point behind_ball = Point(4, 0) -
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));

    EXPECT_TRUE(PenaltyKickTacticFSM::evaluatePenaltyShot(std::nullopt, world.field(),
                                                          world.ball().position(), shooter));
}

TEST(PenaltyKickTacticFSMTest, enemy_goalie_offset_left_no_viable_shot)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().friendlyPenaltyMark(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), 0.2);
    Vector behind_ball_direction =
        -(world.ball().position() - world.field().enemyGoalpostNeg()).normalize();
    Point behind_ball = world.ball().position() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, 0), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world.updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world.updateFriendlyTeamState(friendly);

    EXPECT_FALSE(PenaltyKickTacticFSM::evaluatePenaltyShot(
        std::optional<Robot>(enemy_goalie), world.field(), world.ball().position(), shooter));
}

TEST(PenaltyKickTacticFSMTest, enemy_goalie_offset_right_no_viable_shot)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world,
                                        world.field().enemyGoalCenter() + Vector(-3, 0),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), -0.2);
    Vector behind_ball_direction =
        (world.ball().position() - enemy_goalie_pos).normalize();
    Point behind_ball = world.ball().position() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, +0.2), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world.updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world.updateFriendlyTeamState(friendly);

    EXPECT_FALSE(PenaltyKickTacticFSM::evaluatePenaltyShot(
        std::optional<Robot>(enemy_goalie), world.field(), world.ball().position(), shooter));
}

TEST(PenaltyKickTacticFSMTest, enemy_goalie_right_viable_shot_left)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(4, 0), Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), +0.2);
    Vector behind_ball_direction =
        -(world.ball().position() - world.field().enemyGoalpostNeg()).normalize();
    Point behind_ball = world.ball().position() -
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, +0.2), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world.updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world.updateFriendlyTeamState(friendly);

    EXPECT_TRUE(PenaltyKickTacticFSM::evaluatePenaltyShot(
        std::optional<Robot>(enemy_goalie), world.field(), world.ball().position(), shooter));
}

TEST(PenaltyKickTacticFSMTest, enemy_goalie_left_viable_shot_right)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(4, 0), Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), -0.2);
    Vector behind_ball_direction =
        -(world.ball().position() - world.field().enemyGoalpostPos()).normalize();
    Point behind_ball = world.ball().position() -
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, +0.2), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world.updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world.updateFriendlyTeamState(friendly);

    EXPECT_TRUE(PenaltyKickTacticFSM::evaluatePenaltyShot(
        std::optional<Robot>(enemy_goalie), world.field(), world.ball().position(), shooter));
}

TEST(PenaltyKickTacticFSMTest, no_enemy_goalie_shot_position)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().friendlyPenaltyMark(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    EXPECT_EQ(PenaltyKickTacticFSM::evaluateNextShotPosition(std::nullopt, world.field()),
              world.field().enemyGoalCenter());
}

TEST(PenaltyKickTacticFSMTest, enemy_goalie_left_shot_right)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().friendlyPenaltyMark(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), 0.2);
    Robot enemy_goalie     = Robot(0, enemy_goalie_pos, Vector(0, 0), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Point shot_position = PenaltyKickTacticFSM::evaluateNextShotPosition(
        std::optional<Robot>(enemy_goalie), world.field());
    EXPECT_LE(shot_position.y(), 0);
    EXPECT_EQ(shot_position.x(), world.field().enemyGoalCenter().x());
}

TEST(PenaltyKickTacticFSMTest, enemy_goalie_right_shot_left)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().friendlyPenaltyMark(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), -0.2);
    Robot enemy_goalie     = Robot(0, enemy_goalie_pos, Vector(0, 0), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Point shot_position = PenaltyKickTacticFSM::evaluateNextShotPosition(
        std::optional<Robot>(enemy_goalie), world.field());
    EXPECT_GE(shot_position.y(), 0);
    EXPECT_EQ(shot_position.x(), world.field().enemyGoalCenter().x());
}

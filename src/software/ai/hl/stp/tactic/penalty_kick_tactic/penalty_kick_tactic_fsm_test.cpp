#include "software/ai/hl/stp/tactic/penalty_kick_tactic/penalty_kick_tactic_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(PenaltyKickTacticFSM, test_transitions) {
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().friendlyPenaltyMark(), Timestamp::fromSeconds(0));
    Robot robot = ::TestUtil::createRobotAtPos(world.field().friendlyPenaltyMark());

    FSM<PenaltyKickTacticFSM> fsm;
    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickTacticFSM::InitialState>));

    PenaltyKickTacticFSM::ControlParams control_params{
        .enemy_goalie = std::nullopt
    };

    fsm.process_event(PenaltyKickTacticFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));

    Point position = world.field().enemyGoalCenter() + Vector(-1, 0);
    robot = ::TestUtil::createRobotAtPos(position);
    world = ::TestUtil::setBallPosition(world, position, Timestamp::fromSeconds(1));
    fsm.process_event(PenaltyKickTacticFSM::Update(
        control_params,
        TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));

    fsm.process_event(PenaltyKickTacticFSM::Update(
        control_params,
        TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<KickFSM>));

    world = ::TestUtil::setBallPosition(world, world.field().enemyGoalCenter(), Timestamp::fromSeconds(2));
    world = ::TestUtil::setBallVelocity(world,
        Vector(PenaltyKickTacticFSM::PENALTY_KICK_SHOT_SPEED, 0), Timestamp::fromSeconds(2));
    fsm.process_event(PenaltyKickTacticFSM::Update(
        control_params,
        TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}

TEST(PenaltyKickTacticFSMTest, no_enemy_goalie)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().friendlyPenaltyMark(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Vector behind_ball_direction =
        -(world.ball().position() - world.field().enemyGoalCenter()).normalize();
    Point behind_ball = world.field().friendlyPenaltyMark() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PenaltyKickTactic tactic =
        PenaltyKickTactic(world.ball(), world.field(), std::nullopt, false);

    EXPECT_TRUE(tactic.evaluatePenaltyShot());
}

TEST(PenaltyKickTacticFSMTest, enemy_goalie_offset_left_no_viable_shot)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().friendlyPenaltyMark(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), 0.2);
    Vector behind_ball_direction =
        -(world.ball().position() - world.field().enemyGoalCenter()).normalize();
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

    PenaltyKickTactic tactic = PenaltyKickTactic(
        world.ball(), world.field(), std::optional<Robot>{enemy_goalie}, false);
    tactic.updateRobot(shooter);

    EXPECT_FALSE(tactic.evaluatePenaltyShot());
}

TEST(PenaltyKickTacticFSMTest, enemy_goalie_offset_right_no_viable_shot)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().friendlyPenaltyMark(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), -0.2);
    Vector behind_ball_direction =
        -(world.ball().position() - enemy_goalie_pos).normalize();
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

    PenaltyKickTactic tactic = PenaltyKickTactic(
        world.ball(), world.field(), std::optional<Robot>{enemy_goalie}, false);
    tactic.updateRobot(shooter);

    EXPECT_FALSE(tactic.evaluatePenaltyShot());
}

TEST(PenaltyKickTacticFSMTest, enemy_goalie_right_viable_shot_left)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(4, 0), Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), +0.2);
    Vector behind_ball_direction =
        -(world.ball().position() - world.field().enemyGoalpostNeg()).normalize();
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

    PenaltyKickTactic tactic = PenaltyKickTactic(
        world.ball(), world.field(), std::optional<Robot>{enemy_goalie}, false);
    tactic.updateRobot(shooter);

    EXPECT_TRUE(tactic.evaluatePenaltyShot());
}

TEST(PenaltyKickTacticFSMTest, enemy_goalie_left_viable_shot_right)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, Point(4, 0), Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world.field().enemyGoalCenter().x(), -0.2);
    Vector behind_ball_direction =
        -(world.ball().position() - world.field().enemyGoalpostPos()).normalize();
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

    PenaltyKickTactic tactic = PenaltyKickTactic(
        world.ball(), world.field(), std::optional<Robot>{enemy_goalie}, false);
    tactic.updateRobot(shooter);

    EXPECT_TRUE(tactic.evaluatePenaltyShot());
}

TEST(PenaltyKickTacticFSMTest, no_enemy_goalie_shot_position)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().friendlyPenaltyMark(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    PenaltyKickTactic tactic =
        PenaltyKickTactic(world.ball(), world.field(), std::nullopt, false);

    EXPECT_EQ(tactic.evaluateNextShotPosition(), world.field().enemyGoalCenter());
}

TEST(PenaltyKickTacticFSMTest, enemy_goalie_left_shot_right)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().friendlyPenaltyMark(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos   = Point(world.field().enemyGoalCenter().x(), 0.2);
    Robot enemy_goalie       = Robot(0, enemy_goalie_pos, Vector(0, 0), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PenaltyKickTactic tactic = PenaltyKickTactic(
        world.ball(), world.field(), std::optional<Robot>{enemy_goalie}, false);

    Point shot_position = tactic.evaluateNextShotPosition();
    EXPECT_LE(shot_position.y(), 0);
    EXPECT_EQ(shot_position.x(), world.field().enemyGoalCenter().x());
}

TEST(PenaltyKickTacticFSMTest, enemy_goalie_right_shot_left)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().friendlyPenaltyMark(),
                                        Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos   = Point(world.field().enemyGoalCenter().x(), -0.2);
    Robot enemy_goalie       = Robot(0, enemy_goalie_pos, Vector(0, 0), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PenaltyKickTactic tactic = PenaltyKickTactic(
        world.ball(), world.field(), std::optional<Robot>{enemy_goalie}, false);

    Point shot_position = tactic.evaluateNextShotPosition();
    EXPECT_GE(shot_position.y(), 0);
    EXPECT_EQ(shot_position.x(), world.field().enemyGoalCenter().x());
}

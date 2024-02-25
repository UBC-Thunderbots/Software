#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

// TODO (#2473): fix and re-enable
TEST(PenaltyKickFSM, DISABLED_test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, world->field().friendlyPenaltyMark(),
                                Timestamp::fromSeconds(0));
    Robot robot = ::TestUtil::createRobotAtPos(world->field().friendlyPenaltyMark());

    TbotsProto::AiConfig ai_config;
    FSM<PenaltyKickFSM> fsm{PenaltyKickFSM(), DribbleSkillFSM(), GetBehindBallFSM()};

    PenaltyKickFSM::ControlParams control_params{};

    fsm.process_event(PenaltyKickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleSkillFSM>));

    double shot_x_position =
        ((world->field().totalXLength() / 2) - (world->field().totalXLength() * 1.0 / 3));

    Point position = Point(shot_x_position - 0.1, 0);
    robot          = ::TestUtil::createRobotAtPos(position);
    ::TestUtil::setBallPosition(world, position, Timestamp::fromSeconds(1));
    fsm.process_event(PenaltyKickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleSkillFSM>));

    position = Point(shot_x_position + 0.3, 0);
    robot    = ::TestUtil::createRobotAtPos(position);
    ::TestUtil::setBallPosition(world, position, Timestamp::fromSeconds(2));
    fsm.process_event(PenaltyKickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<KickFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<KickFSM>)>(
        boost::sml::state<GetBehindBallFSM>));

    ::TestUtil::setBallPosition(world, position + Vector(0.1, 0),
                                Timestamp::fromSeconds(2));
    fsm.process_event(PenaltyKickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<KickFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<KickFSM>)>(
        boost::sml::state<KickFSM::KickState>));

    ::TestUtil::setBallPosition(world, world->field().enemyGoalCenter(),
                                Timestamp::fromSeconds(4));
    ::TestUtil::setBallVelocity(world, Vector(5, 0), Timestamp::fromSeconds(4));
    fsm.process_event(PenaltyKickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));

    EXPECT_TRUE(fsm.is(boost::sml::X));
}

TEST(PenaltyKickFSMTest, no_enemy_goalie)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, Point(4, 0), Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Vector behind_ball_direction =
        -(world->ball().position() - world->field().enemyGoalCenter()).normalize();
    Point behind_ball =
        Point(4, 0) - behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                      BALL_MAX_RADIUS_METERS);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));

    EXPECT_TRUE(PenaltyKickFSM::evaluatePenaltyShot(std::nullopt, world->field(),
                                                    world->ball().position(), shooter));
}

TEST(PenaltyKickFSMTest, enemy_goalie_offset_left_no_viable_shot)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, world->field().friendlyPenaltyMark(),
                                Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world->field().enemyGoalCenter().x(), 0.2);
    Vector behind_ball_direction =
        -(world->ball().position() - world->field().enemyGoalpostNeg()).normalize();
    Point behind_ball = world->ball().position() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, 0), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world->updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world->updateFriendlyTeamState(friendly);

    EXPECT_FALSE(PenaltyKickFSM::evaluatePenaltyShot(std::optional<Robot>(enemy_goalie),
                                                     world->field(),
                                                     world->ball().position(), shooter));
}

TEST(PenaltyKickFSMTest, enemy_goalie_offset_right_no_viable_shot)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, world->field().enemyGoalCenter() + Vector(-3, 0),
                                Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world->field().enemyGoalCenter().x(), -0.2);
    Vector behind_ball_direction =
        (world->ball().position() - enemy_goalie_pos).normalize();
    Point behind_ball = world->ball().position() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, +0.2), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world->updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world->updateFriendlyTeamState(friendly);

    EXPECT_FALSE(PenaltyKickFSM::evaluatePenaltyShot(std::optional<Robot>(enemy_goalie),
                                                     world->field(),
                                                     world->ball().position(), shooter));
}

TEST(PenaltyKickFSMTest, enemy_goalie_right_viable_shot_left)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, Point(4, 0), Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world->field().enemyGoalCenter().x(), +0.2);
    Vector behind_ball_direction =
        -(world->ball().position() - world->field().enemyGoalpostNeg()).normalize();
    Point behind_ball = world->ball().position() -
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, +0.2), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world->updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world->updateFriendlyTeamState(friendly);

    EXPECT_TRUE(PenaltyKickFSM::evaluatePenaltyShot(std::optional<Robot>(enemy_goalie),
                                                    world->field(),
                                                    world->ball().position(), shooter));
}

TEST(PenaltyKickFSMTest, enemy_goalie_left_viable_shot_right)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, Point(4, 0), Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world->field().enemyGoalCenter().x(), -0.2);
    Vector behind_ball_direction =
        -(world->ball().position() - world->field().enemyGoalpostPos()).normalize();
    Point behind_ball = world->ball().position() -
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, +0.2), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world->updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world->updateFriendlyTeamState(friendly);

    EXPECT_TRUE(PenaltyKickFSM::evaluatePenaltyShot(std::optional<Robot>(enemy_goalie),
                                                    world->field(),
                                                    world->ball().position(), shooter));
}

TEST(PenaltyKickFSMTest, no_enemy_goalie_shot_position)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, world->field().friendlyPenaltyMark(),
                                Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    EXPECT_EQ(PenaltyKickFSM::evaluateNextShotPosition(std::nullopt, world->field()),
              world->field().enemyGoalCenter());
}

TEST(PenaltyKickFSMTest, enemy_goalie_left_shot_right)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, world->field().friendlyPenaltyMark(),
                                Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world->field().enemyGoalCenter().x(), 0.2);
    Robot enemy_goalie     = Robot(0, enemy_goalie_pos, Vector(0, 0), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Point shot_position = PenaltyKickFSM::evaluateNextShotPosition(
        std::optional<Robot>(enemy_goalie), world->field());
    EXPECT_LE(shot_position.y(), 0);
    EXPECT_EQ(shot_position.x(), world->field().enemyGoalCenter().x());
}

TEST(PenaltyKickFSMTest, enemy_goalie_right_shot_left)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, world->field().friendlyPenaltyMark(),
                                Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world->field().enemyGoalCenter().x(), -0.2);
    Robot enemy_goalie     = Robot(0, enemy_goalie_pos, Vector(0, 0), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Point shot_position = PenaltyKickFSM::evaluateNextShotPosition(
        std::optional<Robot>(enemy_goalie), world->field());
    EXPECT_GE(shot_position.y(), 0);
    EXPECT_EQ(shot_position.x(), world->field().enemyGoalCenter().x());
}

#include "software/ai/hl/stp/play/ball_placement/ball_placement_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(BallPlacementPlayFSMTest, test_transitions)
{
    int num_tactics = 5;

    World world = ::TestUtil::createBlankTestingWorld();
    // ball starts within the field lines
    world.updateBall(Ball(Point(2, 2), Vector(0, 0), Timestamp::fromSeconds(0)));

    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    Point ball_placement_point(0, 0);
    game_state.setBallPlacementPoint(ball_placement_point);
    world.updateGameState(game_state);

    TbotsProto::AiConfig ai_config;
    FSM<BallPlacementPlayFSM> fsm(BallPlacementPlayFSM{ai_config});

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::StartState>));

    fsm.process_event(BallPlacementPlayFSM::Update(
        BallPlacementPlayFSM::ControlParams{},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::AlignPlacementState>));
}

TEST(BallPlacementPlayFSMTest, test_kick_off_wall_transitions)
{
    int num_tactics = 5;

    // default field type is DIV_B
    World world = ::TestUtil::createBlankTestingWorld();
    // ball starts outside the field lines, so FSM will enter KickOfWallState
    world.updateBall(Ball(Point(-2.0, 3.2), Vector(0, 0), Timestamp::fromSeconds(0)));

    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    Point ball_placement_point(0, 0);
    game_state.setBallPlacementPoint(ball_placement_point);
    world.updateGameState(game_state);

    TbotsProto::AiConfig ai_config;
    FSM<BallPlacementPlayFSM> fsm(BallPlacementPlayFSM{ai_config});

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::StartState>));

    fsm.process_event(BallPlacementPlayFSM::Update(
        BallPlacementPlayFSM::ControlParams{},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::KickOffWallState>));

    // After the ball is kicked off a wall, it ends up somewhere inside the field lines
    // (but still needs to be moved to the ball placement point)
    world.updateBall(Ball(Point(-1, 2), Vector(1, 1), Timestamp::fromSeconds(1)));

    fsm.process_event(BallPlacementPlayFSM::Update(
        BallPlacementPlayFSM::ControlParams{},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::AlignPlacementState>));
}

TEST(BallPlacementPlayFSMTest, test_kick_off_wall_angle)
{
    TbotsProto::AiConfig ai_config;
    BallPlacementPlayFSM fsm(ai_config);

    Field field           = Field::createField(TbotsProto::FieldType::DIV_B);
    Rectangle field_lines = field.fieldLines();

    // friendly half, ball outside left sideline
    Point start_point(-2.0, 3.2);
    Angle kick_angle = fsm.calculateWallKickoffAngle(start_point, field_lines);
    EXPECT_TRUE(kick_angle.toDegrees() == 45);

    // friendly half, ball outside right sideline
    start_point = Point(-2.0, -3.2);
    kick_angle  = fsm.calculateWallKickoffAngle(start_point, field_lines);
    EXPECT_TRUE(kick_angle.toDegrees() == -45);

    // enemy half, ball outside left sideline
    start_point = Point(2.0, 3.2);
    kick_angle  = fsm.calculateWallKickoffAngle(start_point, field_lines);
    EXPECT_TRUE(kick_angle.toDegrees() == 135);

    // enemy half, ball outside right sideline
    start_point = Point(2.0, -3.2);
    kick_angle  = fsm.calculateWallKickoffAngle(start_point, field_lines);
    EXPECT_TRUE(kick_angle.toDegrees() == -135);

    // friendly half, ball outside friendly goal line (left of goal)
    start_point = Point(-4.7, 1.6);
    kick_angle  = fsm.calculateWallKickoffAngle(start_point, field_lines);
    EXPECT_TRUE(kick_angle.toDegrees() == 135);

    // friendly half, ball outside friendly goal line (right of goal)
    start_point = Point(-4.7, -1.6);
    kick_angle  = fsm.calculateWallKickoffAngle(start_point, field_lines);
    EXPECT_TRUE(kick_angle.toDegrees() == -135);

    // enemy half, ball outside enemy goal line (left of goal)
    start_point = Point(4.7, 1.6);
    kick_angle  = fsm.calculateWallKickoffAngle(start_point, field_lines);
    EXPECT_TRUE(kick_angle.toDegrees() == 45);

    // enemy half, ball outside enemy goal line (right of goal)
    start_point = Point(4.7, -1.6);
    kick_angle  = fsm.calculateWallKickoffAngle(start_point, field_lines);
    EXPECT_TRUE(kick_angle.toDegrees() == -45);
}

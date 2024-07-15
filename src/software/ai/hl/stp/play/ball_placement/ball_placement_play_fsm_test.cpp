#include "software/ai/hl/stp/play/ball_placement/ball_placement_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(BallPlacementPlayFSMTest, test_transitions)
{
    int num_tactics = 5;

    std::shared_ptr<World> world_ptr = ::TestUtil::createBlankTestingWorld();
    // ball starts within the field lines
    world_ptr->updateBall(Ball(Point(2, 2), Vector(0, 0), Timestamp::fromSeconds(0)));

    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    Point ball_placement_point(0, 0);
    game_state.setBallPlacementPoint(ball_placement_point);
    world_ptr->updateGameState(game_state);

    TbotsProto::AiConfig ai_config;
    FSM<BallPlacementPlayFSM> fsm(BallPlacementPlayFSM{ai_config});

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::StartState>));

    fsm.process_event(BallPlacementPlayFSM::Update(
        BallPlacementPlayFSM::ControlParams{},
        PlayUpdate(
            world_ptr, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::AlignPlacementState>));
}

TEST(BallPlacementPlayFSMTest, test_pick_off_wall_transitions)
{
    int num_tactics = 5;

    // default field type is DIV_B
    std::shared_ptr<World> world_ptr = ::TestUtil::createBlankTestingWorld();
    // ball starts outside the field lines, so FSM will try to align for a wall pick off
    world_ptr->updateBall(
        Ball(Point(0, 3.2), Vector(0, 0), Timestamp::fromSeconds(0)));

    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    Point ball_placement_point(0, 0);
    game_state.setBallPlacementPoint(ball_placement_point);
    world_ptr->updateGameState(game_state);

    TbotsProto::AiConfig ai_config;
    FSM<BallPlacementPlayFSM> fsm(BallPlacementPlayFSM{ai_config});

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::StartState>));

    fsm.process_event(BallPlacementPlayFSM::Update(
        BallPlacementPlayFSM::ControlParams{},
        PlayUpdate(
                world_ptr, num_tactics, [](PriorityTacticVector new_tactics) {},
                InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::AlignWallState>));

    ::TestUtil::setFriendlyRobotPositions(world_ptr, {Point(0, 2.8)}, Timestamp::fromSeconds(0));

    fsm.process_event(BallPlacementPlayFSM::Update(
        BallPlacementPlayFSM::ControlParams{},
        PlayUpdate(
                world_ptr, num_tactics, [](PriorityTacticVector new_tactics) {},
                InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::PickOffWallState>));

    // After the robot has completed the pickoff, we expect it to be done waiting
    world_ptr->updateBall(Ball(Point(-2.0, 0), Vector(0, 0), Timestamp::fromSeconds(0)));
    ::TestUtil::setFriendlyRobotPositions(world_ptr, {Point(0, 0)}, Timestamp::fromSeconds(0));

    fsm.process_event(BallPlacementPlayFSM::Update(
        BallPlacementPlayFSM::ControlParams{},
        PlayUpdate(
            world_ptr, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::PickOffWaitState>));

    // After the robot is done waiting, we want to be in align placement

    world_ptr->updateTimestamp(Timestamp::fromSeconds(4));

    fsm.process_event(BallPlacementPlayFSM::Update(
            BallPlacementPlayFSM::ControlParams{},
            PlayUpdate(
                    world_ptr, num_tactics, [](PriorityTacticVector new_tactics) {},
                    InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::AlignPlacementState>));
}

//    TEST(BallPlacementPlayFSMTest, test_pick_off_wall_angle_position)
//    {
//    TbotsProto::AiConfig ai_config;
//    BallPlacementPlayFSM fsm(ai_config);
//
//    Field field           = Field::createField(TbotsProto::FieldType::DIV_B);
//    Rectangle field_lines = field.fieldLines();
//
//    // friendly half, ball outside left sideline
//    Point start_point(-2.0, 3.2);
//    Angle kick_angle = fsm.calculateWallKickoffAngle(start_point, field_lines);
//    EXPECT_TRUE(kick_angle.toDegrees() == 45);
//
//    // friendly half, ball outside right sideline
//    start_point = Point(-2.0, -3.2);
//    kick_angle  = fsm.calculateWallKickoffAngle(start_point, field_lines);
//    EXPECT_TRUE(kick_angle.toDegrees() == -45);
//
//    // enemy half, ball outside left sideline
//    start_point = Point(2.0, 3.2);
//    kick_angle  = fsm.calculateWallKickoffAngle(start_point, field_lines);
//    EXPECT_TRUE(kick_angle.toDegrees() == 135);
//
//    // enemy half, ball outside right sideline
//    start_point = Point(2.0, -3.2);
//    kick_angle  = fsm.calculateWallKickoffAngle(start_point, field_lines);
//    EXPECT_TRUE(kick_angle.toDegrees() == -135);
//
//    // friendly half, ball outside friendly goal line (left of goal)
//    start_point = Point(-4.7, 1.6);
//    kick_angle  = fsm.calculateWallKickoffAngle(start_point, field_lines);
//    EXPECT_TRUE(kick_angle.toDegrees() == 135);
//
//    // friendly half, ball outside friendly goal line (right of goal)
//    start_point = Point(-4.7, -1.6);
//    kick_angle  = fsm.calculateWallKickoffAngle(start_point, field_lines);
//    EXPECT_TRUE(kick_angle.toDegrees() == -135);
//
//    // enemy half, ball outside enemy goal line (left of goal)
//    start_point = Point(4.7, 1.6);
//    kick_angle  = fsm.calculateWallKickoffAngle(start_point, field_lines);
//    EXPECT_TRUE(kick_angle.toDegrees() == 45);
//
//    // enemy half, ball outside enemy goal line (right of goal)
//    start_point = Point(4.7, -1.6);
//    kick_angle  = fsm.calculateWallKickoffAngle(start_point, field_lines);
//    EXPECT_TRUE(kick_angle.toDegrees() == -45);
//}

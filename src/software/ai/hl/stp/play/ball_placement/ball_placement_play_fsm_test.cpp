#include "software/ai/hl/stp/play/ball_placement/ball_placement_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/test_util/test_util.h"

static void update(FSM<BallPlacementPlayFSM>& fsm, std::shared_ptr<World> world_ptr,
                   int num_tactics)
{
    fsm.process_event(BallPlacementPlayFSM::Update(
        BallPlacementPlayFSM::ControlParams{},
        PlayUpdate(
            world_ptr, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));
}

TEST(BallPlacementPlayFSMTest, test_align_placement)
{
    int num_tactics = 5;

    Point ball_point(2, 2);
    std::shared_ptr<World> world_ptr = ::TestUtil::createBlankTestingWorld();
    // ball starts within the field lines
    world_ptr->updateBall(Ball(ball_point, Vector(0, 0), Timestamp::fromSeconds(0)));

    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    Point ball_placement_point(0, 0);
    game_state.setBallPlacementPoint(ball_placement_point);
    world_ptr->updateGameState(game_state);


    FSM<BallPlacementPlayFSM> fsm(
        BallPlacementPlayFSM{std::make_shared<TbotsProto::AiConfig>()});

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::StartState>));

    update(fsm, world_ptr, num_tactics);

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::AlignPlacementState>));

    double ALIGNMENT_VECTOR_LENGTH_M = ROBOT_MAX_RADIUS_METERS * 2.5;
    Vector alignment_vector =
        (ball_placement_point - ball_point).normalize(ALIGNMENT_VECTOR_LENGTH_M);
    Point expected_setup_point = ball_point - alignment_vector;

    ::TestUtil::setFriendlyRobotPositions(world_ptr, {expected_setup_point},
                                          Timestamp::fromSeconds(0));

    update(fsm, world_ptr, num_tactics);

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::PlaceBallState>));

    world_ptr->updateBall(
        Ball(ball_placement_point, Vector(0, 0), Timestamp::fromSeconds(1)));

    update(fsm, world_ptr, num_tactics);
    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::ReleaseBallState>));
}

TEST(BallPlacementPlayFSMTest, test_align_wall)
{
    int num_tactics = 5;

    std::shared_ptr<World> world_ptr = ::TestUtil::createBlankTestingWorld();
    // ball starts outside the field lines
    world_ptr->updateBall(Ball(Point(4.8, 3.3), Vector(0, 0), Timestamp::fromSeconds(0)));

    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    Point ball_placement_point(0, 0);
    game_state.setBallPlacementPoint(ball_placement_point);
    world_ptr->updateGameState(game_state);

    FSM<BallPlacementPlayFSM> fsm(
        BallPlacementPlayFSM{std::make_shared<TbotsProto::AiConfig>()});

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::StartState>));

    update(fsm, world_ptr, num_tactics);

    // ball in +X +Y corner
    Angle expected_angle   = Angle::fromDegrees(45);
    Vector approach_vector = Vector::createFromAngle(expected_angle);
    Point expected_pickoff_point =
        Point(4.8, 3.3) - approach_vector.normalize(ROBOT_MAX_RADIUS_METERS * 2.5);

    ::TestUtil::setFriendlyRobotPositions(world_ptr, {expected_pickoff_point},
                                          Timestamp::fromSeconds(0));

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::AlignWallState>));

    update(fsm, world_ptr, num_tactics);

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::PickOffWallState>));
}

TEST(BallPlacementPlayFSMTest, test_back_and_forth)
{
    int num_tactics = 5;

    std::shared_ptr<World> world_ptr = ::TestUtil::createBlankTestingWorld();
    // ball starts outside the field lines
    world_ptr->updateBall(Ball(Point(4.8, 3.3), Vector(0, 0), Timestamp::fromSeconds(0)));

    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    Point ball_placement_point(0, 0);
    game_state.setBallPlacementPoint(ball_placement_point);
    world_ptr->updateGameState(game_state);

    FSM<BallPlacementPlayFSM> fsm(
        BallPlacementPlayFSM{std::make_shared<TbotsProto::AiConfig>()});

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::StartState>));

    update(fsm, world_ptr, num_tactics);
    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::AlignWallState>));

    // put ball back inside field lines
    world_ptr->updateBall(Ball(Point(4.4, 2.9), Vector(0, 0), Timestamp::fromSeconds(0)));

    update(fsm, world_ptr, num_tactics);
    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::AlignPlacementState>));

    // put ball back outside field lines
    world_ptr->updateBall(Ball(Point(4.8, 3.3), Vector(0, 0), Timestamp::fromSeconds(0)));
    update(fsm, world_ptr, num_tactics);

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::AlignWallState>));
}

TEST(BallPlacementPlayFSMTest, test_reset_when_ball_dropped_during_release)
{
    int num_tactics = 5;
    Point ball_placement_point(0, 0);

    std::shared_ptr<World> world_ptr = ::TestUtil::createBlankTestingWorld();
    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    game_state.setBallPlacementPoint(ball_placement_point);
    world_ptr->updateGameState(game_state);

    FSM<BallPlacementPlayFSM> fsm(
        BallPlacementPlayFSM{std::make_shared<TbotsProto::AiConfig>()});

    world_ptr->updateBall(Ball(Point(2, 2), Vector(0, 0), Timestamp::fromSeconds(0)));
    update(fsm, world_ptr, num_tactics);

    Vector alignment_vector =
        (ball_placement_point - Point(2, 2)).normalize(ROBOT_MAX_RADIUS_METERS * 2.5);
    ::TestUtil::setFriendlyRobotPositions(world_ptr, {Point(2, 2) - alignment_vector},
                                          Timestamp::fromSeconds(0));
    update(fsm, world_ptr, num_tactics);

    world_ptr->updateBall(
        Ball(ball_placement_point, Vector(0, 0), Timestamp::fromSeconds(1)));
    update(fsm, world_ptr, num_tactics);

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::ReleaseBallState>));

    // ball bumped away during release
    world_ptr->updateBall(
        Ball(Point(1.0, 1.0), Vector(0.5, 0.5), Timestamp::fromSeconds(2)));
    update(fsm, world_ptr, num_tactics);

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::StartState>));
}

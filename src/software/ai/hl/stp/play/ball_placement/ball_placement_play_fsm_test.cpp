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


    FSM<BallPlacementPlayFSM> fsm(
        BallPlacementPlayFSM{std::make_shared<TbotsProto::AiConfig>()});

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::StartState>));

    fsm.process_event(BallPlacementPlayFSM::Update(
        BallPlacementPlayFSM::ControlParams{},
        PlayUpdate(
            world_ptr, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<BallPlacementPlayFSM::AlignPlacementState>));
}

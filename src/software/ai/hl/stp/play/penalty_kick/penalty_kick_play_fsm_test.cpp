#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(PenaltyKickPlayFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::HALT);
    world.updateGameState(game_state);
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    world.updateGameState(game_state);
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
    world.updateGameState(game_state);

    int num_tactics = 5;

    TbotsProto::AiConfig ai_config;

    FSM<PenaltyKickPlayFSM> fsm(PenaltyKickPlayFSM{ai_config});

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickPlayFSM::SetupPositionState>));

    fsm.process_event(PenaltyKickPlayFSM::Update(
        PenaltyKickPlayFSM::ControlParams{},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickPlayFSM::SetupPositionState>));

    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    world.updateGameState(game_state);

    fsm.process_event(PenaltyKickPlayFSM::Update(
        PenaltyKickPlayFSM::ControlParams{},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickPlayFSM::PerformKickState>));

    fsm.process_event(PenaltyKickPlayFSM::Update(
        PenaltyKickPlayFSM::ControlParams{},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickPlayFSM::PerformKickState>));
}

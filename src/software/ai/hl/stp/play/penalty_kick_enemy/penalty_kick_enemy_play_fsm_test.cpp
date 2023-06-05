#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(PenaltyKickEnemyPlayFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::HALT);
    world.updateGameState(game_state);
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    world.updateGameState(game_state);
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_THEM);
    world.updateGameState(game_state);

    TbotsProto::AiConfig ai_config;

    int num_tactics = 5;
    std::shared_ptr<GoalieTactic> goalie_tactic = std::make_shared<GoalieTactic>(ai_config);

    FSM<PenaltyKickEnemyPlayFSM> fsm(PenaltyKickEnemyPlayFSM{ai_config});

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickEnemyPlayFSM::SetupPositionState>));

    fsm.process_event(PenaltyKickEnemyPlayFSM::Update(
        PenaltyKickEnemyPlayFSM::ControlParams{goalie_tactic},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickEnemyPlayFSM::SetupPositionState>));

    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    world.updateGameState(game_state);

    fsm.process_event(PenaltyKickEnemyPlayFSM::Update(
        PenaltyKickEnemyPlayFSM::ControlParams{goalie_tactic},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickEnemyPlayFSM::DefendKickState>));

    fsm.process_event(PenaltyKickEnemyPlayFSM::Update(
        PenaltyKickEnemyPlayFSM::ControlParams{goalie_tactic},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickEnemyPlayFSM::DefendKickState>));
}

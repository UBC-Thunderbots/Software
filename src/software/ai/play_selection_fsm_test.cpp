#include "software/ai/play_selection_fsm.h"

#include <gtest/gtest.h>

#include <exception>

#include "software/ai/hl/stp/play/halt_play.h"
#include "software/test_util/test_util.h"
#include "software/world/world.h"

class PlaySelectionFSMTest : public ::testing::Test
{
   protected:
    TbotsProto::AiConfig ai_config;
    std::unique_ptr<FSM<PlaySelectionFSM>> fsm =
        std::make_unique<FSM<PlaySelectionFSM>>(PlaySelectionFSM{ai_config});
    GameState game_state;
};

TEST_F(PlaySelectionFSMTest, test_transition_out_of_penalty_kick)
{
    std::unique_ptr<Play> current_play = std::make_unique<HaltPlay>(ai_config);

    // Start in halt
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Halt>));
    EXPECT_EQ("HaltPlay", objectTypeName(*current_play));

    // Stop
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Stop>));
    EXPECT_EQ("StopPlay", objectTypeName(*current_play));

    // Penalty kick preparation
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("PenaltyKickPlay", objectTypeName(*current_play));

    // Normal start
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    EXPECT_TRUE(game_state.isReadyState());
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("PenaltyKickPlay", objectTypeName(*current_play));

    // Playing
    game_state.updateRefereeCommand(RefereeCommand::HALT);
    game_state.updateRefereeCommand(RefereeCommand::FORCE_START);
    EXPECT_TRUE(game_state.isPlaying());
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Playing>));
    EXPECT_EQ("OffensePlay", objectTypeName(*current_play));
}

TEST_F(PlaySelectionFSMTest, test_transition_out_of_penalty_kick_enemy_when_goal_conceded)
{
    std::unique_ptr<Play> current_play = std::make_unique<HaltPlay>(ai_config);

    // Start in halt
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Halt>));
    EXPECT_EQ("HaltPlay", objectTypeName(*current_play));

    // Stop
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Stop>));
    EXPECT_EQ("StopPlay", objectTypeName(*current_play));

    // Penalty kick preparation
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_THEM);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(game_state.isTheirPenalty());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("PenaltyKickEnemyPlay", objectTypeName(*current_play));

    // Normal start
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("PenaltyKickEnemyPlay", objectTypeName(*current_play));

    // Goal conceded
    game_state.updateRefereeCommand(RefereeCommand::GOAL_THEM);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(game_state.isStopped());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Stop>));
    EXPECT_EQ("StopPlay", objectTypeName(*current_play));

    // Kickoff preparation
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_KICKOFF_US);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(game_state.isSetupState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("KickoffFriendlyPlay", objectTypeName(*current_play));

    // Normal start
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("KickoffFriendlyPlay", objectTypeName(*current_play));

    // Ball is kicked and restart state is cleared, enter playing state
    game_state.setRestartCompleted();
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(game_state.isPlaying());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Playing>));
    EXPECT_EQ("OffensePlay", objectTypeName(*current_play));
}

TEST_F(PlaySelectionFSMTest,
       test_transition_out_of_penalty_kick_enemy_when_no_goal_conceded)
{
    std::unique_ptr<Play> current_play = std::make_unique<HaltPlay>(ai_config);

    // Start in halt
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Halt>));
    EXPECT_EQ("HaltPlay", objectTypeName(*current_play));

    // Stop
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Stop>));
    EXPECT_EQ("StopPlay", objectTypeName(*current_play));

    // Penalty kick preparation
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_THEM);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(game_state.isTheirPenalty());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("PenaltyKickEnemyPlay", objectTypeName(*current_play));

    // Normal start
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("PenaltyKickEnemyPlay", objectTypeName(*current_play));

    // Stop because no goal
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(game_state.isStopped());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Stop>));
    EXPECT_EQ("StopPlay", objectTypeName(*current_play));

    // Free kick
    game_state.updateRefereeCommand(RefereeCommand::DIRECT_FREE_US);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(game_state.isOurDirectFree());
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("FreeKickPlay", objectTypeName(*current_play));

    // Ball is kicked and restart state is cleared, enter playing state
    game_state.setRestartCompleted();
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state, ai_config));
    EXPECT_TRUE(game_state.isPlaying());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Playing>));
    EXPECT_EQ("OffensePlay", objectTypeName(*current_play));
}

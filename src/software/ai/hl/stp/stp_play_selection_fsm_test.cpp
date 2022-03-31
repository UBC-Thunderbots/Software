#include <gtest/gtest.h>

#include <exception>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/stp.h"
#include "software/test_util/test_util.h"
#include "software/world/world.h"

TEST(PlaySelectionFSMTest, test_transitions)
{
    auto ai_config = std::make_shared<ThunderbotsConfig>()->getAiConfig();
    std::unique_ptr<Play> current_play(std::make_unique<HaltPlay>(ai_config));
    std::unique_ptr<FSM<PlaySelectionFSM>> fsm(
        std::make_unique<FSM<PlaySelectionFSM>>(PlaySelectionFSM{ai_config}));
    GameState game_state;

    // Start in halt
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state));

    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Halt>));
    EXPECT_EQ("HaltPlay", objectTypeName(*current_play));

    // Stop
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state));

    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Stop>));
    EXPECT_EQ("StopPlay", objectTypeName(*current_play));

    // Penalty kick preparation
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("PenaltyKickPlay", objectTypeName(*current_play));

    // Normal start
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    EXPECT_TRUE(game_state.isReadyState());
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("PenaltyKickPlay", objectTypeName(*current_play));

    // Playing
    game_state.updateRefereeCommand(RefereeCommand::HALT);
    game_state.updateRefereeCommand(RefereeCommand::FORCE_START);
    EXPECT_TRUE(game_state.isPlaying());
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::unique_ptr<Play> play) { current_play = std::move(play); },
        game_state));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Playing>));
    EXPECT_EQ("OffensePlay", objectTypeName(*current_play));
}

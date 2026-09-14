#include "software/ai/play_selection_fsm.h"

#include <gtest/gtest.h>

#include <exception>

#include "software/ai/hl/stp/play/halt_play/halt_play.h"
#include "software/test_util/test_util.h"
#include "software/world/world.h"

class PlaySelectionFSMTest : public ::testing::Test
{
   protected:
    TbotsProto::AiConfig ai_config;
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr =
        std::make_shared<TbotsProto::AiConfig>(ai_config);
    std::unique_ptr<FSM<PlaySelectionFSM>> fsm =
        std::make_unique<FSM<PlaySelectionFSM>>(PlaySelectionFSM{ai_config_ptr});
    GameState game_state;

    Play& selectedPlay() const
    {
        return static_cast<const PlaySelectionFSM&>(*fsm).getSelectedPlay();
    }

    void update()
    {
        fsm->process_event(PlaySelectionFSM::Update(game_state, ai_config));
    }
};

TEST_F(PlaySelectionFSMTest, test_override_preserves_ai_selection_state)
{
    // Stop
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    update();
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Stop>));
    EXPECT_EQ("StopPlay", objectTypeName(selectedPlay()));

    // Override stop play with halt
    fsm->process_event(
        PlaySelectionFSM::Override(std::make_unique<HaltPlay>(ai_config_ptr)));
    EXPECT_EQ("HaltPlay", objectTypeName(selectedPlay()));

    // Play selection should continue while overridden
    game_state.updateRefereeCommand(RefereeCommand::FORCE_START);
    update();
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Playing>));
    EXPECT_EQ("HaltPlay", objectTypeName(selectedPlay()));

    // Remove override, should show new selected play from previous step
    fsm->process_event(PlaySelectionFSM::Override(nullptr));
    EXPECT_EQ("OffensePlay", objectTypeName(selectedPlay()));
}

TEST_F(PlaySelectionFSMTest, test_reset_clears_override_and_resets_selection_state)
{
    // Start
    game_state.updateRefereeCommand(RefereeCommand::FORCE_START);
    update();
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Playing>));

    // Override offense play with halt
    fsm->process_event(
        PlaySelectionFSM::Override(std::make_unique<HaltPlay>(ai_config_ptr)));
    EXPECT_EQ("HaltPlay", objectTypeName(selectedPlay()));

    // Play selection should be reset
    fsm->process_event(PlaySelectionFSM::Reset(nullptr));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Halt>));
    EXPECT_EQ("OffensePlay", objectTypeName(selectedPlay()));
}

TEST_F(PlaySelectionFSMTest, test_transition_out_of_penalty_kick)
{
    // Start in halt
    update();
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Halt>));
    EXPECT_EQ("HaltPlay", objectTypeName(selectedPlay()));

    // Stop
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    update();
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Stop>));
    EXPECT_EQ("StopPlay", objectTypeName(selectedPlay()));

    // Penalty kick preparation
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
    update();
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("PenaltyKickPlay", objectTypeName(selectedPlay()));

    // Normal start
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    EXPECT_TRUE(game_state.isReadyState());
    update();
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("PenaltyKickPlay", objectTypeName(selectedPlay()));

    // Playing
    game_state.updateRefereeCommand(RefereeCommand::HALT);
    game_state.updateRefereeCommand(RefereeCommand::FORCE_START);
    EXPECT_TRUE(game_state.isPlaying());
    update();
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Playing>));
    EXPECT_EQ("OffensePlay", objectTypeName(selectedPlay()));
}

TEST_F(PlaySelectionFSMTest, test_transition_out_of_penalty_kick_enemy_when_goal_conceded)
{
    // Start in halt
    update();
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Halt>));
    EXPECT_EQ("HaltPlay", objectTypeName(selectedPlay()));

    // Stop
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    update();
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Stop>));
    EXPECT_EQ("StopPlay", objectTypeName(selectedPlay()));

    // Penalty kick preparation
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_THEM);
    update();
    EXPECT_TRUE(game_state.isTheirPenalty());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("PenaltyKickEnemyPlay", objectTypeName(selectedPlay()));

    // Normal start
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    update();
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("PenaltyKickEnemyPlay", objectTypeName(selectedPlay()));

    // Goal conceded
    game_state.updateRefereeCommand(RefereeCommand::GOAL_THEM);
    update();
    EXPECT_TRUE(game_state.isStopped());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Stop>));
    EXPECT_EQ("StopPlay", objectTypeName(selectedPlay()));

    // Kickoff preparation
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_KICKOFF_US);
    update();
    EXPECT_TRUE(game_state.isSetupState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("KickoffFriendlyPlay", objectTypeName(selectedPlay()));

    // Normal start
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    update();
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("KickoffFriendlyPlay", objectTypeName(selectedPlay()));

    // Ball is kicked and restart state is cleared, enter playing state
    game_state.setRestartCompleted();
    update();
    EXPECT_TRUE(game_state.isPlaying());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Playing>));
    EXPECT_EQ("OffensePlay", objectTypeName(selectedPlay()));
}

TEST_F(PlaySelectionFSMTest,
       test_transition_out_of_penalty_kick_enemy_when_no_goal_conceded)
{
    // Start in halt
    update();
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Halt>));
    EXPECT_EQ("HaltPlay", objectTypeName(selectedPlay()));

    // Stop
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    update();
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Stop>));
    EXPECT_EQ("StopPlay", objectTypeName(selectedPlay()));

    // Penalty kick preparation
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_THEM);
    update();
    EXPECT_TRUE(game_state.isTheirPenalty());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("PenaltyKickEnemyPlay", objectTypeName(selectedPlay()));

    // Normal start
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    update();
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("PenaltyKickEnemyPlay", objectTypeName(selectedPlay()));

    // Stop because no goal
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    update();
    EXPECT_TRUE(game_state.isStopped());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Stop>));
    EXPECT_EQ("StopPlay", objectTypeName(selectedPlay()));

    // Free kick
    game_state.updateRefereeCommand(RefereeCommand::DIRECT_FREE_US);
    update();
    EXPECT_TRUE(game_state.isOurDirectFree());
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("FreeKickPlay", objectTypeName(selectedPlay()));

    // Ball is kicked and restart state is cleared, enter playing state
    game_state.setRestartCompleted();
    update();
    EXPECT_TRUE(game_state.isPlaying());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Playing>));
    EXPECT_EQ("OffensePlay", objectTypeName(selectedPlay()));
}

TEST_F(PlaySelectionFSMTest, test_transition_between_ball_placement_and_free_kick)
{
    // Start in halt
    update();
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Halt>));
    EXPECT_EQ("HaltPlay", objectTypeName(selectedPlay()));

    // Stop
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    update();
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Stop>));
    EXPECT_EQ("StopPlay", objectTypeName(selectedPlay()));

    // Friendly ball placement
    game_state.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_US);
    update();
    EXPECT_TRUE(game_state.isOurBallPlacement());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("BallPlacementPlay", objectTypeName(selectedPlay()));

    // Friendly free kick
    game_state.updateRefereeCommand(RefereeCommand::DIRECT_FREE_US);
    update();
    EXPECT_TRUE(game_state.isOurDirectFree());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("FreeKickPlay", objectTypeName(selectedPlay()));

    // Enemy ball placement
    game_state.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_THEM);
    update();
    EXPECT_TRUE(game_state.isTheirBallPlacement());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("EnemyBallPlacementPlay", objectTypeName(selectedPlay()));

    // Enemy free kick
    game_state.updateRefereeCommand(RefereeCommand::DIRECT_FREE_THEM);
    update();
    EXPECT_TRUE(game_state.isTheirDirectFree());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlay>));
    EXPECT_EQ("EnemyFreeKickPlay", objectTypeName(selectedPlay()));

    // Ball is kicked and restart state is cleared, enter playing state
    game_state.setRestartCompleted();
    update();
    EXPECT_TRUE(game_state.isPlaying());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::Playing>));
    EXPECT_EQ("OffensePlay", objectTypeName(selectedPlay()));
}

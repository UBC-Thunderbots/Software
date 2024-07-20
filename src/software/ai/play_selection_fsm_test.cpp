#include "software/ai/play_selection_fsm.h"

#include <gtest/gtest.h>

#include <exception>

#include "software/ai/hl/stp/play/halt_play.h"
#include "software/test_util/test_util.h"
#include "software/world/world.h"

class PlaySelectionFSMTest : public ::testing::Test
{
   protected:
    std::shared_ptr<Strategy> strategy =
        std::make_shared<Strategy>(TbotsProto::AiConfig());
    std::shared_ptr<FSM<PlaySelectionFSM>> fsm =
        std::make_unique<FSM<PlaySelectionFSM>>(PlaySelectionFSM{strategy});
    std::shared_ptr<World> world_ptr = TestUtil::createBlankTestingWorld();
    GameState game_state;
};

TEST_F(PlaySelectionFSMTest, test_transition_out_of_penalty_kick)
{
    std::shared_ptr<Play> current_play = std::make_unique<HaltPlay>(strategy);

    // Start in halt
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::HaltState>));
    EXPECT_EQ("HaltPlay", objectTypeName(*current_play));

    // Stop
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::StopState>));
    EXPECT_EQ("StopPlay", objectTypeName(*current_play));

    // Penalty kick preparation
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlayState>));
    EXPECT_EQ("PenaltyKickPlay", objectTypeName(*current_play));

    // Normal start
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    world_ptr->updateGameState(game_state);
    EXPECT_TRUE(game_state.isReadyState());
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlayState>));
    EXPECT_EQ("PenaltyKickPlay", objectTypeName(*current_play));

    // Playing
    game_state.updateRefereeCommand(RefereeCommand::HALT);
    game_state.updateRefereeCommand(RefereeCommand::FORCE_START);
    world_ptr->updateGameState(game_state);
    EXPECT_TRUE(game_state.isPlaying());
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::OffensePlayState>));
}

TEST_F(PlaySelectionFSMTest, test_transition_out_of_penalty_kick_enemy_when_goal_conceded)
{
    std::shared_ptr<Play> current_play = std::make_unique<HaltPlay>(strategy);

    // Start in halt
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::HaltState>));
    EXPECT_EQ("HaltPlay", objectTypeName(*current_play));

    // Stop
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::StopState>));
    EXPECT_EQ("StopPlay", objectTypeName(*current_play));

    // Penalty kick preparation
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_THEM);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isTheirPenalty());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlayState>));
    EXPECT_EQ("PenaltyKickEnemyPlay", objectTypeName(*current_play));

    // Normal start
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlayState>));
    EXPECT_EQ("PenaltyKickEnemyPlay", objectTypeName(*current_play));

    // Goal conceded
    game_state.updateRefereeCommand(RefereeCommand::GOAL_THEM);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isStopped());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::StopState>));
    EXPECT_EQ("StopPlay", objectTypeName(*current_play));

    // Kickoff preparation
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_KICKOFF_US);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isSetupState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlayState>));
    EXPECT_EQ("KickoffFriendlyPlay", objectTypeName(*current_play));

    // Normal start
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlayState>));
    EXPECT_EQ("KickoffFriendlyPlay", objectTypeName(*current_play));

    // Ball is kicked and restart state is cleared, enter playing state
    game_state.setRestartCompleted();
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isPlaying());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::OffensePlayState>));
}

TEST_F(PlaySelectionFSMTest,
       test_transition_out_of_penalty_kick_enemy_when_no_goal_conceded)
{
    std::shared_ptr<Play> current_play = std::make_unique<HaltPlay>(strategy);

    // Start in halt
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::HaltState>));
    EXPECT_EQ("HaltPlay", objectTypeName(*current_play));

    // Stop
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::StopState>));
    EXPECT_EQ("StopPlay", objectTypeName(*current_play));

    // Penalty kick preparation
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_THEM);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isTheirPenalty());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlayState>));
    EXPECT_EQ("PenaltyKickEnemyPlay", objectTypeName(*current_play));

    // Normal start
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlayState>));
    EXPECT_EQ("PenaltyKickEnemyPlay", objectTypeName(*current_play));

    // Stop because no goal
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isStopped());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::StopState>));
    EXPECT_EQ("StopPlay", objectTypeName(*current_play));

    // Free kick
    game_state.updateRefereeCommand(RefereeCommand::DIRECT_FREE_US);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isOurDirectFree());
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlayState>));
    EXPECT_EQ("FreeKickPlay", objectTypeName(*current_play));

    // Ball is kicked and restart state is cleared, enter playing state
    game_state.setRestartCompleted();
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isPlaying());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::OffensePlayState>));
}

TEST_F(PlaySelectionFSMTest, test_transition_between_ball_placement_and_free_kick)
{
    std::shared_ptr<Play> current_play = std::make_shared<HaltPlay>(strategy);

    // Start in halt
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::HaltState>));
    EXPECT_EQ("HaltPlay", objectTypeName(*current_play));

    // Stop
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    world_ptr->updateGameState(game_state);
    ;
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::StopState>));
    EXPECT_EQ("StopPlay", objectTypeName(*current_play));

    // Friendly ball placement
    game_state.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_US);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isOurBallPlacement());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlayState>));
    EXPECT_EQ("BallPlacementPlay", objectTypeName(*current_play));

    // Friendly free kick
    game_state.updateRefereeCommand(RefereeCommand::DIRECT_FREE_US);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isOurDirectFree());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlayState>));
    EXPECT_EQ("FreeKickPlay", objectTypeName(*current_play));

    // Enemy ball placement
    game_state.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_THEM);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isTheirBallPlacement());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlayState>));
    EXPECT_EQ("EnemyBallPlacementPlay", objectTypeName(*current_play));

    // Enemy free kick
    game_state.updateRefereeCommand(RefereeCommand::DIRECT_FREE_THEM);
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isTheirDirectFree());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::SetPlayState>));
    EXPECT_EQ("EnemyFreeKickPlay", objectTypeName(*current_play));

    // Ball is kicked and restart state is cleared, enter playing state
    game_state.setRestartCompleted();
    world_ptr->updateGameState(game_state);
    fsm->process_event(PlaySelectionFSM::Update(
        [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));
    EXPECT_TRUE(game_state.isPlaying());
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::OffensePlayState>));
    EXPECT_EQ("OffensePlay", objectTypeName(*current_play));
}

TEST_F(PlaySelectionFSMTest, test_transition_between_stagnant_defence_to_offense)
{
    auto world = TestUtil::createBlankTestingWorld();

    // Position all robots in the friendly half.
    auto friendly_team = TestUtil::setRobotPositionsHelper(
            world->friendlyTeam(), {Point(-0.45, 0)}, Timestamp::fromSeconds(0));
    auto enemy_team = TestUtil::setRobotPositionsHelper(
            world->enemyTeam(), {Point(-0.55, 0)}, Timestamp::fromSeconds(0));
    world->updateFriendlyTeamState(friendly_team);
    world->updateEnemyTeamState(enemy_team);

    std::shared_ptr<Play> current_play = std::make_shared<HaltPlay>(strategy);

    world->updateBall(Ball({-0.5, 0}, {0, 0}, Timestamp::fromSeconds(0)));
    // Position all bots in the enemy half.
//    friendly_team = TestUtil::setRobotPositionsHelper(
//            world->friendlyTeam(), {Point(0.45, 0)}, Timestamp::fromSeconds(3));
//    enemy_team = TestUtil::setRobotPositionsHelper(world->enemyTeam(), {Point(0.55, 0)},
//                                                   Timestamp::fromSeconds(1));
    world->updateFriendlyTeamState(friendly_team);
    world->updateEnemyTeamState(enemy_team);
    world->updateBall(Ball({-0.53, 0}, {0, 0}, Timestamp::fromSeconds(0)));
    world->updateBall(Ball({-0.53, 0}, {0, 0}, Timestamp::fromSeconds(0.5)));

    fsm->process_event(PlaySelectionFSM::Update(
            [&current_play](std::shared_ptr<Play> play) { current_play = play; }, world));    // Move ball near enemy bot for a period of time.
    // Enemy team should have clear possession.

    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::DefensePlayState>));
    EXPECT_TRUE(fsm->is(boost::sml::state<PlaySelectionFSM::OffensePlayState>));
}
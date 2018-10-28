#include "ai/world/game_state.h"

#include <gtest/gtest.h>


TEST(GameStateTest, test_construct) {
    GameState state;
    EXPECT_TRUE(state.halt());
    EXPECT_FALSE(state.playing());
}

TEST(GameStateTest, test_halt) {
    GameState state;
    state.updateRefboxGameState(RefboxGameState::HALT);
    EXPECT_TRUE(state.halt());
    EXPECT_FALSE(state.playing());
}

TEST(GameStateTest, test_stop) {
    GameState state;
    state.updateRefboxGameState(RefboxGameState::STOP);
    EXPECT_TRUE(state.stopped());
}

TEST(GameStateTest, test_our_kickoff_restart) {
    GameState state;
    state.updateRefboxGameState(RefboxGameState::PREPARE_KICKOFF_US);
    EXPECT_TRUE(state.inSetupState());
    EXPECT_TRUE(state.ourKickoff());
    EXPECT_TRUE(state.isOurRestart());
    state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    EXPECT_TRUE(state.ourKickoff());
    EXPECT_TRUE(state.playing());
}

TEST(GameStateTest, test_their_kickoff_restart) {
    GameState state;
    state.updateRefboxGameState(RefboxGameState::PREPARE_KICKOFF_THEM);
    EXPECT_TRUE(state.theirKickoff());
    EXPECT_TRUE(state.stayAwayFromBall());
    EXPECT_TRUE(state.stayOnSide());
    EXPECT_FALSE(state.isOurRestart());
    state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    EXPECT_TRUE(state.theirKickoff());
    EXPECT_TRUE(state.playing());
}

TEST(GameStateTest, test_our_indirect_restart) {
    GameState state;
    state.updateRefboxGameState(RefboxGameState::INDIRECT_FREE_US);
    EXPECT_TRUE(state.ourIndirect());
    EXPECT_TRUE(state.setupRestart());
    EXPECT_TRUE(state.isOurRestart());
    EXPECT_TRUE(state.ourFreeKick());
    state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    EXPECT_TRUE(state.ourIndirect());
    EXPECT_TRUE(state.playing());
}

TEST(GameStateTest, test_their_indirect_restart) {
    GameState state;
    state.updateRefboxGameState(RefboxGameState::INDIRECT_FREE_THEM);
    EXPECT_TRUE(state.theirIndirect());
    EXPECT_TRUE(state.setupRestart());
    EXPECT_TRUE(state.stayAwayFromBall());
    EXPECT_FALSE(state.isOurRestart());
    EXPECT_FALSE(state.ourFreeKick());
    state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    EXPECT_TRUE(state.theirIndirect());
    EXPECT_TRUE(state.playing());
}

TEST(GameStateTest, test_our_placement) {
    GameState state;
    state.updateRefboxGameState(RefboxGameState::BALL_PLACEMENT_US);
    EXPECT_TRUE(state.ballPlacement());
    EXPECT_TRUE(state.setupRestart());
    EXPECT_TRUE(state.isOurRestart());
}

TEST(GameStateTest, test_their_placement) {
    GameState state;
    state.updateRefboxGameState(RefboxGameState::BALL_PLACEMENT_THEM);
    EXPECT_TRUE(state.ballPlacement());
    EXPECT_FALSE(state.isOurRestart());
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
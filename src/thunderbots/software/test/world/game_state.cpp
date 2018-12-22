#include "ai/world/game_state.h"

#include <gtest/gtest.h>


TEST(GameStateTest, test_construct)
{
    GameState state;
    EXPECT_TRUE(state.isHalted());
    EXPECT_FALSE(state.isPlaying());
}

TEST(GameStateTest, test_halt)
{
    GameState state;
    state.updateRefboxGameState(RefboxGameState::HALT);
    EXPECT_TRUE(state.isHalted());
    EXPECT_FALSE(state.isPlaying());
}

TEST(GameStateTest, test_stop)
{
    GameState state;
    state.updateRefboxGameState(RefboxGameState::STOP);
    EXPECT_TRUE(state.isStopped());
}

TEST(GameStateTest, test_our_kickoff_restart)
{
    GameState state;
    state.updateRefboxGameState(RefboxGameState::PREPARE_KICKOFF_US);
    EXPECT_TRUE(state.isSetupState());
    EXPECT_TRUE(state.isOurKickoff());
    EXPECT_TRUE(state.isOurRestart());
    state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    EXPECT_TRUE(state.isOurKickoff());
    EXPECT_TRUE(state.isPlaying());
}

TEST(GameStateTest, test_their_kickoff_restart)
{
    GameState state;
    state.updateRefboxGameState(RefboxGameState::PREPARE_KICKOFF_THEM);
    EXPECT_TRUE(state.isTheirKickoff());
    EXPECT_TRUE(state.stayAwayFromBall());
    EXPECT_TRUE(state.stayOnSide());
    EXPECT_FALSE(state.isOurRestart());
    state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    EXPECT_TRUE(state.isTheirKickoff());
    EXPECT_TRUE(state.isPlaying());
}

TEST(GameStateTest, test_our_indirect_restart)
{
    GameState state;
    state.updateRefboxGameState(RefboxGameState::INDIRECT_FREE_US);
    EXPECT_TRUE(state.isOurIndirect());
    EXPECT_TRUE(state.isSetupRestart());
    EXPECT_TRUE(state.isOurRestart());
    EXPECT_TRUE(state.isOurFreeKick());
    state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    EXPECT_TRUE(state.isOurIndirect());
    EXPECT_TRUE(state.isPlaying());
}

TEST(GameStateTest, test_their_indirect_restart)
{
    GameState state;
    state.updateRefboxGameState(RefboxGameState::INDIRECT_FREE_THEM);
    EXPECT_TRUE(state.isTheirIndirectFree());
    EXPECT_TRUE(state.isSetupRestart());
    EXPECT_TRUE(state.stayAwayFromBall());
    EXPECT_FALSE(state.isOurRestart());
    EXPECT_FALSE(state.isOurFreeKick());
    state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    EXPECT_TRUE(state.isTheirIndirectFree());
    EXPECT_TRUE(state.isPlaying());
}

TEST(GameStateTest, test_our_placement)
{
    GameState state;
    state.updateRefboxGameState(RefboxGameState::BALL_PLACEMENT_US);
    EXPECT_TRUE(state.isBallPlacement());
    EXPECT_TRUE(state.isSetupRestart());
    EXPECT_TRUE(state.isOurRestart());
}

TEST(GameStateTest, test_their_placement)
{
    GameState state;
    state.updateRefboxGameState(RefboxGameState::BALL_PLACEMENT_THEM);
    EXPECT_TRUE(state.isBallPlacement());
    EXPECT_FALSE(state.isOurRestart());
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
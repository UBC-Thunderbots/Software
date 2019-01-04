#include <gtest/gtest.h>

#include "test/test_util/test_util.h"
#include "util/refbox_constants.h"

class GameStateTest : public ::testing::TestWithParam<RefboxGameState>
{
   protected:
    void SetUp() override {}
};

TEST_P(GameStateTest, test_all_state_transitions_to_halt)
{
    GameState game_state;
    game_state.updateRefboxGameState(GetParam());
    game_state.updateRefboxGameState(RefboxGameState::HALT);
    EXPECT_TRUE(game_state.isHalted());
    EXPECT_FALSE(game_state.isStopped());
    EXPECT_FALSE(game_state.isKickoff());
    EXPECT_FALSE(game_state.isPenalty());
    EXPECT_FALSE(game_state.isBallPlacement());
    EXPECT_FALSE(game_state.isOurRestart());
    EXPECT_FALSE(game_state.isDirectFree());
    EXPECT_FALSE(game_state.isIndirectFree());
    EXPECT_FALSE(game_state.isOurKickoff());
    EXPECT_FALSE(game_state.isOurDirect());
    EXPECT_FALSE(game_state.isOurPenalty());
    EXPECT_FALSE(game_state.isOurDirect());
    EXPECT_FALSE(game_state.isOurIndirect());
    EXPECT_FALSE(game_state.isOurFreeKick());
    EXPECT_FALSE(game_state.isOurPlacement());
    EXPECT_FALSE(game_state.isTheirKickoff());
    EXPECT_FALSE(game_state.isTheirPenalty());
    EXPECT_FALSE(game_state.isTheirDirectFree());
    EXPECT_FALSE(game_state.isTheirIndirectFree());
    EXPECT_FALSE(game_state.isTheirFreeKick());
    EXPECT_FALSE(game_state.isTheirBallPlacement());
    EXPECT_FALSE(game_state.isSetupRestart());
    EXPECT_FALSE(game_state.isSetupState());
    EXPECT_FALSE(game_state.isReadyState());
    EXPECT_FALSE(game_state.canKick());
    // ignoring things that are irrelevant to the HALT state for now
    // such as stayAwayFromBall
}

INSTANTIATE_TEST_CASE_P(AllStates, GameStateTest,
                        ::testing::ValuesIn(::Test::TestUtil::getAllRefboxGameStates()));

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
};
#include <gtest/gtest.h>

#include "software/sensor_fusion/refbox_data.h"
#include "software/test_util/test_util.h"

TEST(GameStateTest, default_constructor)
{
    GameState game_state;

    EXPECT_EQ(RefboxGameState::HALT, game_state.getRefboxGameState());
    EXPECT_EQ(GameState::RestartReason::NONE, game_state.getRestartReason());
    EXPECT_FALSE(game_state.isOurRestart());

    EXPECT_TRUE(game_state.isHalted());
    EXPECT_FALSE(game_state.isStopped());
    EXPECT_FALSE(game_state.isPlaying());
    EXPECT_FALSE(game_state.isKickoff());
    EXPECT_FALSE(game_state.isPenalty());
    EXPECT_FALSE(game_state.isBallPlacement());
    EXPECT_FALSE(game_state.isOurRestart());
    EXPECT_FALSE(game_state.isDirectFree());
    EXPECT_FALSE(game_state.isIndirectFree());
    EXPECT_FALSE(game_state.isOurKickoff());
    EXPECT_FALSE(game_state.isOurPenalty());
    EXPECT_FALSE(game_state.isOurDirectFree());
    EXPECT_FALSE(game_state.isOurIndirectFree());
    EXPECT_FALSE(game_state.isOurFreeKick());
    EXPECT_FALSE(game_state.isOurBallPlacement());
    EXPECT_FALSE(game_state.isTheirKickoff());
    EXPECT_FALSE(game_state.isTheirPenalty());
    EXPECT_FALSE(game_state.isTheirDirectFree());
    EXPECT_FALSE(game_state.isTheirIndirectFree());
    EXPECT_FALSE(game_state.isTheirFreeKick());
    EXPECT_FALSE(game_state.isTheirBallPlacement());
    EXPECT_FALSE(game_state.isSetupRestart());
    EXPECT_FALSE(game_state.isReadyState());
    EXPECT_FALSE(game_state.canKick());
    EXPECT_TRUE(game_state.stayAwayFromBall());
    EXPECT_FALSE(game_state.stayOnSide());
    EXPECT_FALSE(game_state.stayBehindPenaltyLine());
}

TEST(GameStateTest, equality)
{
    GameState gameState1;
    GameState gameState2;
    EXPECT_EQ(gameState1, gameState2);
    EXPECT_EQ(gameState2, gameState1);
    EXPECT_EQ(gameState1, gameState1);
}

TEST(GameStateTest, equality_diff_state)
{
    GameState gameState1;
    GameState gameState2;
    gameState2.state = GameState::STOP;
    EXPECT_NE(gameState1, gameState2);
}

TEST(GameStateTest, equality_diff_restart_reason)
{
    GameState gameState1;
    GameState gameState2;
    gameState2.restart_reason = GameState::PENALTY;
    EXPECT_NE(gameState1, gameState2);
}

TEST(GameStateTest, equality_diff_game_state)
{
    GameState gameState1;
    GameState gameState2;
    gameState2.game_state = RefboxGameState::STOP;
    EXPECT_NE(gameState1, gameState2);
}

TEST(GameStateTest, equality_diff_ball_state)
{
    GameState gameState1;
    GameState gameState2;
    gameState2.state = GameState::READY;
    gameState2.updateBall(
        Ball(Point(100, 100), Vector(20, 0), Timestamp::fromSeconds(0)));
    EXPECT_NE(gameState1, gameState2);
}

TEST(GameStateTest, equality_diff_our_restart)
{
    GameState gameState1;
    GameState gameState2;
    gameState2.our_restart = true;
    EXPECT_NE(gameState1, gameState2);
}

TEST(GameStateTest, equality_diff_placement_point)
{
    GameState gameState1;
    GameState gameState2;
    gameState2.setBallPlacementPoint(Point(100, 100));
    EXPECT_NE(gameState1, gameState2);
}

// tuple of start state, update state, end state, our_restart, restart reason
typedef std::tuple<RefboxGameState, RefboxGameState, RefboxGameState, bool,
                   GameState::RestartReason>
    StateTransitionTestParams;

class GameStateTransitionTest : public ::testing::TestWithParam<StateTransitionTestParams>
{
};

TEST_P(GameStateTransitionTest, test_state_transitions)
{
    RefboxGameState start_state             = std::get<0>(GetParam());
    RefboxGameState update_state            = std::get<1>(GetParam());
    RefboxGameState end_state               = std::get<2>(GetParam());
    bool our_restart                        = std::get<3>(GetParam());
    GameState::RestartReason restart_reason = std::get<4>(GetParam());
    Ball ball(Point(), Vector(), Timestamp::fromSeconds(0));

    GameState game_state;
    game_state.updateRefboxGameState(start_state);
    game_state.updateRefboxGameState(update_state);
    game_state.updateBall(ball);
    EXPECT_EQ(game_state.getRefboxGameState(), end_state);
    EXPECT_EQ(game_state.isOurRestart(), our_restart);
    EXPECT_EQ(game_state.getRestartReason(), restart_reason);
}

/**
 * Returns a tuple of start, update, end states, our restart?, and restart reason
 *
 * @param start refbox game state that the game state class is initially updated with
 * @param update refbox game state that the game state class is updated with
 * @param end refbox game state that the game state class is expected to end with
 * @param our_restart whether it is our restart
 * @param restart_reason the reason for restart, e.g. indirect free kick for us
 *
 * @return tuple of start, update, end states, our restart?, and restart reason
 */

#define STATE_TRANSITION_PARAMS(start, update, end, our_restart, restart_reason)         \
    std::make_tuple<RefboxGameState, RefboxGameState, RefboxGameState, bool,             \
                    GameState::RestartReason>(                                           \
        RefboxGameState::start, RefboxGameState::update, RefboxGameState::end,           \
        our_restart, GameState::RestartReason::restart_reason)

INSTANTIATE_TEST_CASE_P(
    All, GameStateTransitionTest,
    ::testing::Values(
        STATE_TRANSITION_PARAMS(HALT, STOP, STOP, false, NONE),
        // transitions to HALT
        STATE_TRANSITION_PARAMS(STOP, HALT, HALT, false, NONE),
        STATE_TRANSITION_PARAMS(PREPARE_KICKOFF_US, HALT, HALT, false, NONE),
        STATE_TRANSITION_PARAMS(PREPARE_KICKOFF_THEM, HALT, HALT, false, NONE),
        STATE_TRANSITION_PARAMS(PREPARE_PENALTY_US, HALT, HALT, false, NONE),
        STATE_TRANSITION_PARAMS(PREPARE_PENALTY_THEM, HALT, HALT, false, NONE),
        STATE_TRANSITION_PARAMS(DIRECT_FREE_US, HALT, HALT, false, NONE),
        STATE_TRANSITION_PARAMS(DIRECT_FREE_THEM, HALT, HALT, false, NONE),
        STATE_TRANSITION_PARAMS(TIMEOUT_US, HALT, HALT, false, NONE),
        STATE_TRANSITION_PARAMS(TIMEOUT_THEM, HALT, HALT, false, NONE),
        STATE_TRANSITION_PARAMS(GOAL_US, HALT, HALT, false, NONE),
        STATE_TRANSITION_PARAMS(GOAL_THEM, HALT, HALT, false, NONE),
        STATE_TRANSITION_PARAMS(BALL_PLACEMENT_US, HALT, HALT, false, NONE),
        STATE_TRANSITION_PARAMS(BALL_PLACEMENT_THEM, HALT, HALT, false, NONE),
        // transitions to STOP
        STATE_TRANSITION_PARAMS(PREPARE_KICKOFF_US, STOP, STOP, false, NONE),
        STATE_TRANSITION_PARAMS(PREPARE_KICKOFF_THEM, STOP, STOP, false, NONE),
        STATE_TRANSITION_PARAMS(PREPARE_PENALTY_US, STOP, STOP, false, NONE),
        STATE_TRANSITION_PARAMS(PREPARE_PENALTY_THEM, STOP, STOP, false, NONE),
        STATE_TRANSITION_PARAMS(DIRECT_FREE_US, STOP, STOP, false, NONE),
        STATE_TRANSITION_PARAMS(DIRECT_FREE_THEM, STOP, STOP, false, NONE),
        STATE_TRANSITION_PARAMS(TIMEOUT_US, STOP, STOP, false, NONE),
        STATE_TRANSITION_PARAMS(TIMEOUT_THEM, STOP, STOP, false, NONE),
        STATE_TRANSITION_PARAMS(GOAL_US, STOP, STOP, false, NONE),
        STATE_TRANSITION_PARAMS(GOAL_THEM, STOP, STOP, false, NONE),
        STATE_TRANSITION_PARAMS(BALL_PLACEMENT_US, STOP, STOP, false, NONE),
        STATE_TRANSITION_PARAMS(BALL_PLACEMENT_THEM, STOP, STOP, false, NONE),
        // transitions from STOP to a restart state
        STATE_TRANSITION_PARAMS(STOP, PREPARE_KICKOFF_US, PREPARE_KICKOFF_US, true,
                                KICKOFF),
        STATE_TRANSITION_PARAMS(STOP, PREPARE_KICKOFF_THEM, PREPARE_KICKOFF_THEM, false,
                                KICKOFF),
        STATE_TRANSITION_PARAMS(STOP, PREPARE_PENALTY_US, PREPARE_PENALTY_US, true,
                                PENALTY),
        STATE_TRANSITION_PARAMS(STOP, PREPARE_PENALTY_THEM, PREPARE_PENALTY_THEM, false,
                                PENALTY),
        STATE_TRANSITION_PARAMS(STOP, DIRECT_FREE_US, DIRECT_FREE_US, true, DIRECT),
        STATE_TRANSITION_PARAMS(STOP, DIRECT_FREE_THEM, DIRECT_FREE_THEM, false, DIRECT),
        STATE_TRANSITION_PARAMS(STOP, INDIRECT_FREE_US, INDIRECT_FREE_US, true, INDIRECT),
        STATE_TRANSITION_PARAMS(STOP, INDIRECT_FREE_THEM, INDIRECT_FREE_THEM, false,
                                INDIRECT),
        STATE_TRANSITION_PARAMS(STOP, TIMEOUT_US, TIMEOUT_US, false, NONE),
        STATE_TRANSITION_PARAMS(STOP, TIMEOUT_THEM, TIMEOUT_THEM, false, NONE),
        STATE_TRANSITION_PARAMS(STOP, BALL_PLACEMENT_US, BALL_PLACEMENT_US, true,
                                BALL_PLACEMENT),
        STATE_TRANSITION_PARAMS(STOP, BALL_PLACEMENT_THEM, BALL_PLACEMENT_THEM, false,
                                BALL_PLACEMENT),
        // a restart state transitioning to NORMAL_START should not clear the restart. We
        // should do that ourselves either when the ball starts moving, or when we kick
        // the ball.
        STATE_TRANSITION_PARAMS(PREPARE_KICKOFF_US, NORMAL_START, NORMAL_START, true,
                                KICKOFF),
        STATE_TRANSITION_PARAMS(PREPARE_KICKOFF_THEM, NORMAL_START, NORMAL_START, false,
                                KICKOFF),
        STATE_TRANSITION_PARAMS(PREPARE_PENALTY_US, NORMAL_START, NORMAL_START, true,
                                PENALTY),
        STATE_TRANSITION_PARAMS(PREPARE_PENALTY_THEM, NORMAL_START, NORMAL_START, false,
                                PENALTY),
        STATE_TRANSITION_PARAMS(DIRECT_FREE_US, NORMAL_START, NORMAL_START, true, DIRECT),
        STATE_TRANSITION_PARAMS(DIRECT_FREE_THEM, NORMAL_START, NORMAL_START, false,
                                DIRECT),
        STATE_TRANSITION_PARAMS(INDIRECT_FREE_US, NORMAL_START, NORMAL_START, true,
                                INDIRECT),
        STATE_TRANSITION_PARAMS(INDIRECT_FREE_THEM, NORMAL_START, NORMAL_START, false,
                                INDIRECT),
        // transitions to a GOAL state
        STATE_TRANSITION_PARAMS(NORMAL_START, GOAL_US, GOAL_US, false, NONE),
        STATE_TRANSITION_PARAMS(NORMAL_START, GOAL_THEM, GOAL_THEM, false, NONE),
        // transition to FORCE_START
        STATE_TRANSITION_PARAMS(NORMAL_START, FORCE_START, FORCE_START, false, NONE)));

class GameStatePredicateTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        allRefboxGameStates = ::TestUtil::getAllRefboxGameStates();
    }
    std::vector<RefboxGameState> allRefboxGameStates;
};

/**
 * Test if the given predicate returns true for all the given states, and false otherwise
 *
 *  @param predicate the predicate to test, e.g. isHalted
 *  @param ... list of refbox game states the given predicate is true for
 */
#define PREDICATE_TEST(predicate, ...)                                                   \
    TEST_F(GameStatePredicateTest, predicate##_test)                                     \
    {                                                                                    \
        Ball ball(Point(), Vector(), Timestamp::fromSeconds(0));                         \
        std::set<RefboxGameState> true_states = {__VA_ARGS__};                           \
        for (auto refbox_game_state : allRefboxGameStates)                               \
        {                                                                                \
            GameState game_state;                                                        \
            game_state.updateRefboxGameState(refbox_game_state);                         \
            game_state.updateBall(ball);                                                 \
            if (true_states.find(refbox_game_state) != true_states.end())                \
            {                                                                            \
                EXPECT_TRUE(game_state.predicate())                                      \
                    << "Expected " << #predicate << " to be true for state "             \
                    << refbox_game_state << std::endl;                                   \
            }                                                                            \
            else                                                                         \
            {                                                                            \
                EXPECT_FALSE(game_state.predicate())                                     \
                    << "Expected " << #predicate << " to be false for state "            \
                    << refbox_game_state << std::endl;                                   \
            }                                                                            \
        }                                                                                \
    }

PREDICATE_TEST(isHalted, RefboxGameState::HALT, RefboxGameState::TIMEOUT_US,
               RefboxGameState::TIMEOUT_THEM)
PREDICATE_TEST(isStopped, RefboxGameState::STOP, RefboxGameState::GOAL_US,
               RefboxGameState::GOAL_THEM)
// PLAYING state must be manually set after a transition from a restart state to
// NORMAL_START
PREDICATE_TEST(isPlaying, RefboxGameState::FORCE_START)
PREDICATE_TEST(isKickoff, RefboxGameState::PREPARE_KICKOFF_US,
               RefboxGameState::PREPARE_KICKOFF_THEM)
PREDICATE_TEST(isPenalty, RefboxGameState::PREPARE_PENALTY_US,
               RefboxGameState::PREPARE_PENALTY_THEM)
PREDICATE_TEST(isBallPlacement, RefboxGameState::BALL_PLACEMENT_US,
               RefboxGameState::BALL_PLACEMENT_THEM)
// isOurRestart tested above already
PREDICATE_TEST(isDirectFree, RefboxGameState::DIRECT_FREE_US,
               RefboxGameState::DIRECT_FREE_THEM)
PREDICATE_TEST(isIndirectFree, RefboxGameState::INDIRECT_FREE_US,
               RefboxGameState::INDIRECT_FREE_THEM)
PREDICATE_TEST(isOurKickoff, RefboxGameState::PREPARE_KICKOFF_US)
PREDICATE_TEST(isOurPenalty, RefboxGameState::PREPARE_PENALTY_US)
PREDICATE_TEST(isOurDirectFree, RefboxGameState::DIRECT_FREE_US)
PREDICATE_TEST(isOurIndirectFree, RefboxGameState::INDIRECT_FREE_US)
PREDICATE_TEST(isOurBallPlacement, RefboxGameState::BALL_PLACEMENT_US)
PREDICATE_TEST(isOurFreeKick, RefboxGameState::DIRECT_FREE_US,
               RefboxGameState::INDIRECT_FREE_US)
PREDICATE_TEST(isTheirKickoff, RefboxGameState::PREPARE_KICKOFF_THEM)
PREDICATE_TEST(isTheirPenalty, RefboxGameState::PREPARE_PENALTY_THEM)
PREDICATE_TEST(isTheirDirectFree, RefboxGameState::DIRECT_FREE_THEM)
PREDICATE_TEST(isTheirFreeKick, RefboxGameState::DIRECT_FREE_THEM,
               RefboxGameState::INDIRECT_FREE_THEM)
PREDICATE_TEST(isTheirBallPlacement, RefboxGameState::BALL_PLACEMENT_THEM)
PREDICATE_TEST(
    isSetupRestart, RefboxGameState::PREPARE_KICKOFF_US,
    RefboxGameState::PREPARE_KICKOFF_THEM, RefboxGameState::BALL_PLACEMENT_US,
    RefboxGameState::BALL_PLACEMENT_THEM, RefboxGameState::DIRECT_FREE_US,
    RefboxGameState::DIRECT_FREE_THEM, RefboxGameState::INDIRECT_FREE_US,
    RefboxGameState::INDIRECT_FREE_THEM,
    // NORMAL_START is a ready state until the restart is cleared when the ball moves
    RefboxGameState::NORMAL_START, RefboxGameState::PREPARE_PENALTY_US,
    RefboxGameState::PREPARE_PENALTY_THEM)
PREDICATE_TEST(isSetupState, RefboxGameState::PREPARE_KICKOFF_US,
               RefboxGameState::PREPARE_KICKOFF_THEM, RefboxGameState::BALL_PLACEMENT_US,
               RefboxGameState::BALL_PLACEMENT_THEM, RefboxGameState::PREPARE_PENALTY_US,
               RefboxGameState::PREPARE_PENALTY_THEM)
PREDICATE_TEST(isReadyState, RefboxGameState::NORMAL_START,
               RefboxGameState::DIRECT_FREE_THEM, RefboxGameState::INDIRECT_FREE_THEM,
               RefboxGameState::DIRECT_FREE_US, RefboxGameState::INDIRECT_FREE_US)
// canKick needs to be tested with a proper restart sequence
PREDICATE_TEST(canKick, RefboxGameState::FORCE_START, RefboxGameState::DIRECT_FREE_US,
               RefboxGameState::INDIRECT_FREE_US)
PREDICATE_TEST(stayOnSide, RefboxGameState::PREPARE_KICKOFF_THEM)
PREDICATE_TEST(stayBehindPenaltyLine, RefboxGameState::PREPARE_PENALTY_THEM,
               RefboxGameState::PREPARE_PENALTY_US)

class GameStateRestartTest : public ::testing::Test
{
};

TEST_F(GameStateRestartTest, kickoff_us_restart_test)
{
    GameState game_state;
    Ball ball(Point(), Vector(), Timestamp::fromSeconds(0));

    RefboxGameState restart_type = RefboxGameState::PREPARE_KICKOFF_US;

    // STOP -> restart_type is how restarts occur during games
    game_state.updateRefboxGameState(RefboxGameState::STOP);
    game_state.updateRefboxGameState(restart_type);
    game_state.updateBall(ball);

    // verify game_state is in the correct state
    EXPECT_TRUE(game_state.isSetupState());
    EXPECT_TRUE(game_state.isKickoff());
    EXPECT_TRUE(game_state.isOurRestart());
    EXPECT_TRUE(game_state.isOurKickoff());

    // restart_type -> NORMAL_START happens next
    game_state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    game_state.updateBall(ball);

    // verify state again
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(game_state.isKickoff());
    EXPECT_TRUE(game_state.isOurRestart());
    EXPECT_TRUE(game_state.isOurKickoff());

    // restart state is cleared when the ball is kicked, enter regular
    // playing state
    game_state.setRestartCompleted();
    EXPECT_TRUE(game_state.isPlaying());
    EXPECT_FALSE(game_state.isKickoff());
}

TEST_F(GameStateRestartTest, kickoff_them_restart_test)
{
    GameState game_state;
    Ball ball(Point(), Vector(), Timestamp::fromSeconds(0));

    RefboxGameState restart_type = RefboxGameState::PREPARE_KICKOFF_THEM;

    // STOP -> restart_type is how restarts occur during games
    game_state.updateRefboxGameState(RefboxGameState::STOP);
    game_state.updateRefboxGameState(restart_type);
    game_state.updateBall(ball);

    // verify game_state is in the correct state
    EXPECT_TRUE(game_state.isSetupState());
    EXPECT_TRUE(game_state.isKickoff());
    EXPECT_FALSE(game_state.isOurRestart());
    EXPECT_TRUE(game_state.isTheirKickoff());

    // restart_type -> NORMAL_START happens next
    game_state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    game_state.updateBall(ball);

    // verify state again
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(game_state.isKickoff());
    EXPECT_FALSE(game_state.isOurRestart());
    EXPECT_TRUE(game_state.isTheirKickoff());

    // restart state is cleared when the ball is kicked, enter regular
    // playing state
    game_state.setRestartCompleted();
    EXPECT_TRUE(game_state.isPlaying());
    EXPECT_FALSE(game_state.isKickoff());
}

TEST_F(GameStateRestartTest, penalty_us_restart_test)
{
    GameState game_state;
    Ball ball(Point(), Vector(), Timestamp::fromSeconds(0));

    RefboxGameState restart_type = RefboxGameState::PREPARE_PENALTY_US;

    // STOP -> restart_type is how restarts occur during games
    game_state.updateRefboxGameState(RefboxGameState::STOP);
    game_state.updateRefboxGameState(restart_type);
    game_state.updateBall(ball);

    // verify game_state is in the correct state
    EXPECT_TRUE(game_state.isSetupState());
    EXPECT_TRUE(game_state.isPenalty());
    EXPECT_TRUE(game_state.isOurRestart());
    EXPECT_TRUE(game_state.isOurPenalty());

    // restart_type -> NORMAL_START happens next
    game_state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    game_state.updateBall(ball);

    // verify state again
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(game_state.isPenalty());
    EXPECT_TRUE(game_state.isOurRestart());
    EXPECT_TRUE(game_state.isOurPenalty());

    // restart state is cleared when the ball is kicked, enter regular
    // playing state
    game_state.setRestartCompleted();
    EXPECT_TRUE(game_state.isPlaying());
    EXPECT_FALSE(game_state.isPenalty());
}

TEST_F(GameStateRestartTest, penalty_them_restart_test)
{
    GameState game_state;
    Ball ball(Point(), Vector(), Timestamp::fromSeconds(0));

    RefboxGameState restart_type = RefboxGameState::PREPARE_PENALTY_THEM;

    // STOP -> restart_type is how restarts occur during games
    game_state.updateRefboxGameState(RefboxGameState::STOP);
    game_state.updateRefboxGameState(restart_type);
    game_state.updateBall(ball);

    // verify game_state is in the correct state
    EXPECT_TRUE(game_state.isSetupState());
    EXPECT_TRUE(game_state.isPenalty());
    EXPECT_FALSE(game_state.isOurRestart());
    EXPECT_TRUE(game_state.isTheirPenalty());

    // restart_type -> NORMAL_START happens next
    game_state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    game_state.updateBall(ball);

    // verify state again
    EXPECT_TRUE(game_state.isReadyState());
    EXPECT_TRUE(game_state.isPenalty());
    EXPECT_FALSE(game_state.isOurRestart());
    EXPECT_TRUE(game_state.isTheirPenalty());

    // restart state is cleared when the ball is kicked, enter regular
    // playing state
    game_state.setRestartCompleted();
    EXPECT_TRUE(game_state.isPlaying());
    EXPECT_FALSE(game_state.isPenalty());
}

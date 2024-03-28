#include <gtest/gtest.h>

#include "proto/message_translation/tbots_protobuf.h"
#include "software/test_util/test_util.h"

TEST(GameStateTest, test_get_name_of_referee_command)
{
    for (unsigned int i = 0; i < sizeRefereeCommand(); i++)
    {
        try
        {
            RefereeCommand state = static_cast<RefereeCommand>(i);
            toString(state);
        }
        catch (std::invalid_argument &)
        {
            ADD_FAILURE() << "Unable to get a name for referee command " << i
                          << std::endl;
        }
        catch (std::exception &)
        {
            ADD_FAILURE()
                << "Unexpected exception thrown while trying to get the Refere Command for "
                << i << std::endl;
        }
    }
}

TEST(GameStateTest, default_constructor)
{
    GameState game_state;

    EXPECT_EQ(RefereeCommand::HALT, game_state.getRefereeCommand());
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

TEST(GameStateTest, construct_with_protobuf)
{
    GameState original_game_state;
    original_game_state.updateRefereeCommand(RefereeCommand::DIRECT_FREE_US);
    auto proto_game_state = createGameStateProto(original_game_state);
    GameState proto_converted_game_state(*proto_game_state);

    EXPECT_EQ(original_game_state, proto_converted_game_state);
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
    gameState2.updateRefereeCommand(RefereeCommand::NORMAL_START);
    EXPECT_NE(gameState1, gameState2);
}

TEST(GameStateTest, equality_diff_restart_reason)
{
    GameState gameState1;
    GameState gameState2;
    gameState1.updateRefereeCommand(RefereeCommand::INDIRECT_FREE_THEM);
    gameState2.updateRefereeCommand(RefereeCommand::DIRECT_FREE_THEM);
    EXPECT_NE(gameState1, gameState2);
}

TEST(GameStateTest, equality_diff_game_state)
{
    GameState gameState1;
    GameState gameState2;
    gameState2.updateRefereeCommand(RefereeCommand::STOP);
    EXPECT_NE(gameState1, gameState2);
}

TEST(GameStateTest, equality_diff_ball_state)
{
    GameState gameState1;
    GameState gameState2;
    gameState2.updateRefereeCommand(RefereeCommand::NORMAL_START);
    gameState2.updateBall(
        Ball(Point(100, 100), Vector(20, 0), Timestamp::fromSeconds(0)));
    EXPECT_NE(gameState1, gameState2);
}

TEST(GameStateTest, equality_diff_our_restart)
{
    GameState gameState1;
    GameState gameState2;
    gameState1.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_US);
    gameState2.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_THEM);
    EXPECT_NE(gameState1, gameState2);
}

TEST(GameStateTest, equality_diff_placement_point)
{
    GameState gameState1;
    GameState gameState2;
    gameState2.setBallPlacementPoint(Point(100, 100));
    EXPECT_NE(gameState1, gameState2);
}

TEST(GetBallPlacementTest, get_ball_placement_no_ball_set)
{
    GameState game_state;
    EXPECT_EQ(game_state.getBallPlacementPoint(), std::nullopt);
}

TEST(GetBallPlacementTest, get_ball_placement_set)
{
    GameState game_state;

    game_state.setBallPlacementPoint(Point(15, 36));
    EXPECT_EQ(game_state.getBallPlacementPoint().value(), Point(15, 36));

    game_state.setBallPlacementPoint(Point(-3, -45));
    EXPECT_EQ(game_state.getBallPlacementPoint().value(), Point(-3, -45));
}

// tuple of start state, update state, end state, our_restart, restart reason
typedef std::tuple<RefereeCommand, RefereeCommand, RefereeCommand, bool,
                   GameState::RestartReason>
    StateTransitionTestParams;

class GameStateTransitionTest : public ::testing::TestWithParam<StateTransitionTestParams>
{
};

TEST_P(GameStateTransitionTest, test_state_transitions)
{
    RefereeCommand start_state              = std::get<0>(GetParam());
    RefereeCommand update_state             = std::get<1>(GetParam());
    RefereeCommand end_state                = std::get<2>(GetParam());
    bool our_restart                        = std::get<3>(GetParam());
    GameState::RestartReason restart_reason = std::get<4>(GetParam());
    Ball ball(Point(), Vector(), Timestamp::fromSeconds(0));

    GameState game_state;
    game_state.updateRefereeCommand(start_state);
    game_state.updateRefereeCommand(update_state);
    game_state.updateBall(ball);
    EXPECT_EQ(game_state.getRefereeCommand(), end_state);
    EXPECT_EQ(game_state.isOurRestart(), our_restart);
    EXPECT_EQ(game_state.getRestartReason(), restart_reason);
}

/**
 * Returns a tuple of start, update, end states, our restart?, and restart reason
 *
 * @param start referee command that the game state class is initially updated with
 * @param update referee command that the game state class is updated with
 * @param end referee command that the game state class is expected to end with
 * @param our_restart whether it is our restart
 * @param restart_reason the reason for restart, e.g. indirect free kick for us
 *
 * @return tuple of start, update, end states, our restart?, and restart reason
 */

#define STATE_TRANSITION_PARAMS(start, update, end, our_restart, restart_reason)         \
    std::make_tuple<RefereeCommand, RefereeCommand, RefereeCommand, bool,                \
                    GameState::RestartReason>(                                           \
        RefereeCommand::start, RefereeCommand::update, RefereeCommand::end, our_restart, \
        GameState::RestartReason::restart_reason)

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
        allRefereeCommands = allValuesRefereeCommand();
    }
    std::vector<RefereeCommand> allRefereeCommands;
};

/**
 * Test if the given predicate returns true for all the given states, and false otherwise
 *
 *  @param predicate the predicate to test, e.g. isHalted
 *  @param ... list of referee commands the given predicate is true for
 */
#define PREDICATE_TEST(predicate, ...)                                                   \
    TEST_F(GameStatePredicateTest, predicate##_test)                                     \
    {                                                                                    \
        Ball ball(Point(), Vector(), Timestamp::fromSeconds(0));                         \
        std::set<RefereeCommand> true_states = {__VA_ARGS__};                            \
        for (auto referee_command : allRefereeCommands)                                  \
        {                                                                                \
            GameState game_state;                                                        \
            game_state.updateRefereeCommand(referee_command);                            \
            game_state.updateBall(ball);                                                 \
            if (true_states.find(referee_command) != true_states.end())                  \
            {                                                                            \
                EXPECT_TRUE(game_state.predicate())                                      \
                    << "Expected " << #predicate << " to be true for state "             \
                    << referee_command << std::endl;                                     \
            }                                                                            \
            else                                                                         \
            {                                                                            \
                EXPECT_FALSE(game_state.predicate())                                     \
                    << "Expected " << #predicate << " to be false for state "            \
                    << referee_command << std::endl;                                     \
            }                                                                            \
        }                                                                                \
    }

PREDICATE_TEST(isHalted, RefereeCommand::HALT, RefereeCommand::TIMEOUT_US,
               RefereeCommand::TIMEOUT_THEM)
PREDICATE_TEST(isStopped, RefereeCommand::STOP, RefereeCommand::GOAL_US,
               RefereeCommand::GOAL_THEM)
// PLAYING state must be manually set after a transition from a restart state to
// NORMAL_START
PREDICATE_TEST(isPlaying, RefereeCommand::FORCE_START)
PREDICATE_TEST(isKickoff, RefereeCommand::PREPARE_KICKOFF_US,
               RefereeCommand::PREPARE_KICKOFF_THEM)
PREDICATE_TEST(isPenalty, RefereeCommand::PREPARE_PENALTY_US,
               RefereeCommand::PREPARE_PENALTY_THEM)
PREDICATE_TEST(isBallPlacement, RefereeCommand::BALL_PLACEMENT_US,
               RefereeCommand::BALL_PLACEMENT_THEM)
// isOurRestart tested above already
PREDICATE_TEST(isDirectFree, RefereeCommand::DIRECT_FREE_US,
               RefereeCommand::DIRECT_FREE_THEM)
PREDICATE_TEST(isIndirectFree, RefereeCommand::INDIRECT_FREE_US,
               RefereeCommand::INDIRECT_FREE_THEM)
PREDICATE_TEST(isOurKickoff, RefereeCommand::PREPARE_KICKOFF_US)
PREDICATE_TEST(isOurPenalty, RefereeCommand::PREPARE_PENALTY_US)
PREDICATE_TEST(isOurDirectFree, RefereeCommand::DIRECT_FREE_US)
PREDICATE_TEST(isOurIndirectFree, RefereeCommand::INDIRECT_FREE_US)
PREDICATE_TEST(isOurBallPlacement, RefereeCommand::BALL_PLACEMENT_US)
PREDICATE_TEST(isOurFreeKick, RefereeCommand::DIRECT_FREE_US,
               RefereeCommand::INDIRECT_FREE_US)
PREDICATE_TEST(isTheirKickoff, RefereeCommand::PREPARE_KICKOFF_THEM)
PREDICATE_TEST(isTheirPenalty, RefereeCommand::PREPARE_PENALTY_THEM)
PREDICATE_TEST(isTheirDirectFree, RefereeCommand::DIRECT_FREE_THEM)
PREDICATE_TEST(isTheirFreeKick, RefereeCommand::DIRECT_FREE_THEM,
               RefereeCommand::INDIRECT_FREE_THEM)
PREDICATE_TEST(isTheirBallPlacement, RefereeCommand::BALL_PLACEMENT_THEM)
PREDICATE_TEST(
    isSetupRestart, RefereeCommand::PREPARE_KICKOFF_US,
    RefereeCommand::PREPARE_KICKOFF_THEM, RefereeCommand::BALL_PLACEMENT_US,
    RefereeCommand::BALL_PLACEMENT_THEM, RefereeCommand::DIRECT_FREE_US,
    RefereeCommand::DIRECT_FREE_THEM, RefereeCommand::INDIRECT_FREE_US,
    RefereeCommand::INDIRECT_FREE_THEM,
    // NORMAL_START is a ready state until the restart is cleared when the ball moves
    RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_PENALTY_US,
    RefereeCommand::PREPARE_PENALTY_THEM)
PREDICATE_TEST(isSetupState, RefereeCommand::PREPARE_KICKOFF_US,
               RefereeCommand::PREPARE_KICKOFF_THEM, RefereeCommand::BALL_PLACEMENT_US,
               RefereeCommand::BALL_PLACEMENT_THEM, RefereeCommand::PREPARE_PENALTY_US,
               RefereeCommand::PREPARE_PENALTY_THEM)
PREDICATE_TEST(isReadyState, RefereeCommand::NORMAL_START,
               RefereeCommand::DIRECT_FREE_THEM, RefereeCommand::INDIRECT_FREE_THEM,
               RefereeCommand::DIRECT_FREE_US, RefereeCommand::INDIRECT_FREE_US)
// canKick needs to be tested with a proper restart sequence
PREDICATE_TEST(canKick, RefereeCommand::FORCE_START, RefereeCommand::DIRECT_FREE_US,
               RefereeCommand::INDIRECT_FREE_US)
PREDICATE_TEST(stayOnSide, RefereeCommand::PREPARE_KICKOFF_THEM)
PREDICATE_TEST(stayBehindPenaltyLine, RefereeCommand::PREPARE_PENALTY_THEM,
               RefereeCommand::PREPARE_PENALTY_US)

class GameStateRestartTest : public ::testing::Test
{
};

TEST_F(GameStateRestartTest, kickoff_us_restart_test)
{
    GameState game_state;
    Ball ball(Point(), Vector(), Timestamp::fromSeconds(0));

    RefereeCommand restart_type = RefereeCommand::PREPARE_KICKOFF_US;

    // STOP -> restart_type is how restarts occur during games
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    game_state.updateRefereeCommand(restart_type);
    game_state.updateBall(ball);

    // verify game_state is in the correct state
    EXPECT_TRUE(game_state.isSetupState());
    EXPECT_TRUE(game_state.isKickoff());
    EXPECT_TRUE(game_state.isOurRestart());
    EXPECT_TRUE(game_state.isOurKickoff());

    // restart_type -> NORMAL_START happens next
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
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

    RefereeCommand restart_type = RefereeCommand::PREPARE_KICKOFF_THEM;

    // STOP -> restart_type is how restarts occur during games
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    game_state.updateRefereeCommand(restart_type);
    game_state.updateBall(ball);

    // verify game_state is in the correct state
    EXPECT_TRUE(game_state.isSetupState());
    EXPECT_TRUE(game_state.isKickoff());
    EXPECT_FALSE(game_state.isOurRestart());
    EXPECT_TRUE(game_state.isTheirKickoff());

    // restart_type -> NORMAL_START happens next
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
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

    RefereeCommand restart_type = RefereeCommand::PREPARE_PENALTY_US;

    // STOP -> restart_type is how restarts occur during games
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    game_state.updateRefereeCommand(restart_type);
    game_state.updateBall(ball);

    // verify game_state is in the correct state
    EXPECT_TRUE(game_state.isSetupState());
    EXPECT_TRUE(game_state.isPenalty());
    EXPECT_TRUE(game_state.isOurRestart());
    EXPECT_TRUE(game_state.isOurPenalty());

    // restart_type -> NORMAL_START happens next
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
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

    RefereeCommand restart_type = RefereeCommand::PREPARE_PENALTY_THEM;

    // STOP -> restart_type is how restarts occur during games
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    game_state.updateRefereeCommand(restart_type);
    game_state.updateBall(ball);

    // verify game_state is in the correct state
    EXPECT_TRUE(game_state.isSetupState());
    EXPECT_TRUE(game_state.isPenalty());
    EXPECT_FALSE(game_state.isOurRestart());
    EXPECT_TRUE(game_state.isTheirPenalty());

    // restart_type -> NORMAL_START happens next
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
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

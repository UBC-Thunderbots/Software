#include <gtest/gtest.h>

#include <variant>

#include "test/ai/hl/stp/test_tactics/move_test_tactic.h"

/**
 * This file contains the unit tests for the Tactic class (NOTE: `Tactic` is virtual, so
 * we use `MoveTestTactic` instead, but we're only testing the generic functionality of
 * the `Tactic` class)
 */

TEST(TacticTest, test_get_assigned_robot_with_no_robot_and_no_params)
{
    MoveTestTactic tactic = MoveTestTactic();
    auto robot            = tactic.getAssignedRobot();

    EXPECT_EQ(robot, std::nullopt);
}

TEST(TacticTest, test_get_assigned_robot_with_no_robot)
{
    MoveTestTactic tactic = MoveTestTactic();
    tactic.updateParams(Point());
    auto robot = tactic.getAssignedRobot();

    EXPECT_EQ(robot, std::nullopt);
}

TEST(TacticTest, test_get_assigned_robot_with_a_robot_assigned)
{
    MoveTestTactic tactic = MoveTestTactic();
    auto robot            = Robot(1, Point(1, 0), Vector(), Angle::threeQuarter(),
                       AngularVelocity::zero(), Timestamp::fromSeconds(0));
    tactic.updateRobot(robot);

    EXPECT_TRUE(tactic.getAssignedRobot());
    EXPECT_EQ(robot, *(tactic.getAssignedRobot()));
}

TEST(TacticTest, test_nullptr_returned_when_no_robot_assigned)
{
    MoveTestTactic tactic = MoveTestTactic();
    tactic.updateParams(Point());

    EXPECT_FALSE(tactic.getNextIntent());
}

TEST(TacticTest, test_tactic_not_done_after_run_with_no_robot_assigned)
{
    MoveTestTactic tactic = MoveTestTactic();
    tactic.updateParams(Point());

    // Run the tactic several times
    for (int i = 0; i < 5; i++)
    {
        tactic.getNextIntent();
    }

    EXPECT_FALSE(tactic.done());
}

TEST(TacticTest, test_tactic_does_not_prematurely_report_done)
{
    // This test is making sure we don't have any bugs with managing our coroutines that
    // cause the tactic to end before we would expect

    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    // Create a MoveTestTactic with the final destination very far from where the robot
    // currently is, so that we know the tactic should not report done
    MoveTestTactic tactic = MoveTestTactic();
    tactic.updateRobot(robot);
    tactic.updateParams(Point(3, 4));

    // Run the Tactic several times, leaving the robot and parameters as is so the
    // tactic should not approach a "done" state
    auto intent_ptr = std::unique_ptr<Intent>{};
    for (int i = 0; i < 10; i++)
    {
        intent_ptr = tactic.getNextIntent();
    }

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(tactic.done());
}

TEST(TacticTest, test_tactic_reports_done_at_same_time_nullptr_returned)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    // Create a MoveTestTactic that wants to move the Robot to where it already is.
    // Therefore we expect the tactic to be done
    MoveTestTactic tactic = MoveTestTactic();
    tactic.updateRobot(robot);
    tactic.updateParams(Point());

    // The tactic should always return an Intent the first time it is run to make sure we
    // are doing the right thing (and just don't happen to be in the "done state" at the
    // time while actually doing something else)
    auto intent_ptr = tactic.getNextIntent();
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(tactic.done());

    // Subsequent calls should return a nullptr since the Tactic is done, and the
    // tactic should also report done()
    intent_ptr = tactic.getNextIntent();
    EXPECT_FALSE(intent_ptr);
    EXPECT_TRUE(tactic.done());
}

TEST(TacticTest, test_tactic_restarts_when_set_to_loop_infinitely)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    // Create a MoveTestTactic that wants to move the Robot to where it already is.
    // Therefore we expect the tactic to be done
    MoveTestTactic tactic = MoveTestTactic(true);
    tactic.updateRobot(robot);
    tactic.updateParams(Point());

    // Even though the Tactic should be done, we expect it to continue returning valid
    // Intents because it will be constantly restarting
    std::unique_ptr<Intent> intent_ptr;
    for (int i = 0; i < 5; i++)
    {
        intent_ptr = tactic.getNextIntent();
        EXPECT_TRUE(intent_ptr);
        EXPECT_FALSE(tactic.done());
    }
}


// Creates a struct which inherits all lambda function given to it and uses their
// Ts::operator(). This can be passed to std::visit to easily write multiple different
// lambdas for each type of motion controller commands below. See
// https://en.cppreference.com/w/cpp/utility/variant/visit for more details.
template <class... Ts>
struct overload : Ts...
{
    using Ts::operator()...;
};
template <class... Ts>
overload(Ts...)->overload<Ts...>;

// Inputs to the gamestate that are not refbox inputs
enum class GameStateInput {
    SET_RESTART_COMPLETED
};

typedef std::tuple<std::vector<std::variant<RefboxGameState, GameStateInput>>, std::vector<AvoidArea>>
    AvoidAreasTestParams;


AvoidAreasTestParams makeParams(std::vector<std::variant<RefboxGameState, GameStateInput>> refbox_states,
                                std::vector<AvoidArea> avoid_areas)
{
    return std::make_tuple<std::vector<std::variant<RefboxGameState, GameStateInput>>, std::vector<AvoidArea>>(
        std::move(refbox_states), std::move(avoid_areas));
}

class TacticAvoidAreasTest : public ::testing::TestWithParam<AvoidAreasTestParams>
{
};

TEST_P(TacticAvoidAreasTest, test_default_avoid_areas)
{
    std::vector<std::variant<RefboxGameState, GameStateInput>> inputs =
            std::get<0>(GetParam());
    std::vector<AvoidArea> expected_avoid_areas     = std::get<1>(GetParam());

    // Set the robot on the tactic
    Robot robot = Robot(0, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveTestTactic tactic = MoveTestTactic(true);
    tactic.updateRobot(robot);

    World world;

    // Process all the inputs to the game state
    std::string input_str;
    for (auto input : inputs){
     std::visit(overload {
            [&](RefboxGameState refbox_game_state) {
                world.mutableGameState().updateRefboxGameState(refbox_game_state);
                std::stringstream ss;
                ss << refbox_game_state << ", ";
                input_str += ss.str();
                },
            [&](GameStateInput game_state_input) {
                switch (game_state_input) {
                    case GameStateInput::SET_RESTART_COMPLETED:
                        world.mutableGameState().setRestartCompleted();
                        input_str += "setRestartCompleted, ";
                        break;
                    default:
                        ADD_FAILURE() << "Unhandled input to gamestate";
                }
                },
        }, input);
    }

    auto next_intent = tactic.getNextIntent(world);
    ASSERT_TRUE(next_intent);

    EXPECT_EQ(expected_avoid_areas, next_intent->getAreasToAvoid())
        << "Failed with input to gamestate: " << input_str;
}

// Parameterized test to check that we return the correct areas to avoid for a
// given refbox state
INSTANTIATE_TEST_CASE_P(
    All, TacticAvoidAreasTest,
    ::testing::Values(
        // Halt
        makeParams({RefboxGameState::HALT},
                   {AvoidArea::HALF_METER_AROUND_BALL, AvoidArea::ENEMY_DEFENSE_AREA,
                    AvoidArea::FRIENDLY_DEFENSE_AREA}),
        // Stop
        makeParams({RefboxGameState::STOP}, {AvoidArea::HALF_METER_AROUND_BALL,
                                             AvoidArea::INFLATED_ENEMY_DEFENSE_AREA,
                                             AvoidArea::FRIENDLY_DEFENSE_AREA}),
        // Our kickoff setup
        makeParams({RefboxGameState::PREPARE_KICKOFF_US},
                   {AvoidArea::ENEMY_DEFENSE_AREA, AvoidArea::FRIENDLY_DEFENSE_AREA}),
        // Our kickoff
        makeParams({RefboxGameState::PREPARE_KICKOFF_US, RefboxGameState::NORMAL_START},
                   {AvoidArea::ENEMY_DEFENSE_AREA, AvoidArea::FRIENDLY_DEFENSE_AREA}),
        // Their kickoff setup
        makeParams({RefboxGameState::PREPARE_KICKOFF_THEM},
                   {AvoidArea::HALF_METER_AROUND_BALL, AvoidArea::ENEMY_DEFENSE_AREA,
                    AvoidArea::FRIENDLY_DEFENSE_AREA}),
        // Their kickoff
        makeParams({RefboxGameState::PREPARE_KICKOFF_THEM, RefboxGameState::NORMAL_START},
                   {AvoidArea::ENEMY_DEFENSE_AREA, AvoidArea::FRIENDLY_DEFENSE_AREA}),
        // Their indirect free kick setup
        makeParams({RefboxGameState::INDIRECT_FREE_THEM},
                   {AvoidArea::HALF_METER_AROUND_BALL, AvoidArea::ENEMY_DEFENSE_AREA,
                    AvoidArea::FRIENDLY_DEFENSE_AREA}),
        // Their indirect free kick
        makeParams({RefboxGameState::INDIRECT_FREE_THEM, RefboxGameState::NORMAL_START},
                   {AvoidArea::ENEMY_DEFENSE_AREA, AvoidArea::FRIENDLY_DEFENSE_AREA}),
        // Our indirect free kick setup
        makeParams({RefboxGameState::INDIRECT_FREE_US},
                   {AvoidArea::INFLATED_ENEMY_DEFENSE_AREA,
                    AvoidArea::FRIENDLY_DEFENSE_AREA}),
        // Our indirect free kick
        makeParams({RefboxGameState::INDIRECT_FREE_US, RefboxGameState::NORMAL_START},
                   {AvoidArea::ENEMY_DEFENSE_AREA, AvoidArea::FRIENDLY_DEFENSE_AREA}),
        // Their direct free kick setup
        makeParams({RefboxGameState::DIRECT_FREE_THEM},
                   {AvoidArea::HALF_METER_AROUND_BALL, AvoidArea::ENEMY_DEFENSE_AREA,
                    AvoidArea::FRIENDLY_DEFENSE_AREA}),
        // Their direct free kick
        makeParams({RefboxGameState::DIRECT_FREE_THEM, RefboxGameState::NORMAL_START},
                   {AvoidArea::ENEMY_DEFENSE_AREA, AvoidArea::FRIENDLY_DEFENSE_AREA}),
        // Our direct free kick setup
        makeParams({RefboxGameState::DIRECT_FREE_US},
                   {AvoidArea::INFLATED_ENEMY_DEFENSE_AREA,
                    AvoidArea::FRIENDLY_DEFENSE_AREA}),
        // Our direct free kick
        makeParams({RefboxGameState::DIRECT_FREE_US, RefboxGameState::NORMAL_START},
                   {AvoidArea::ENEMY_DEFENSE_AREA, AvoidArea::FRIENDLY_DEFENSE_AREA}),
        // Our penalty kick setup
        makeParams({RefboxGameState::PREPARE_PENALTY_US}, {AvoidArea::ENEMY_HALF}),
        // Our penalty kick
        makeParams({RefboxGameState::PREPARE_PENALTY_US, RefboxGameState::NORMAL_START},
                   {AvoidArea::ENEMY_HALF}),
        // Their penalty kick setup
        makeParams({RefboxGameState::PREPARE_PENALTY_THEM}, {AvoidArea::FRIENDLY_HALF}),
        // Their penalty kick
        makeParams({RefboxGameState::PREPARE_PENALTY_THEM, RefboxGameState::NORMAL_START},
                   {AvoidArea::FRIENDLY_HALF})));

TEST(TacticTest, test_and_remove_extra_avoid_areas)
{
    // Test adding and removing extra avoid areas is correctly reflected in the
    // avoid areas produced by the tactic
    Robot robot = Robot(0, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveTestTactic tactic = MoveTestTactic(true);
    tactic.updateRobot(robot);

    tactic.addBlacklistedAvoidArea(AvoidArea::FRIENDLY_HALF);
    tactic.addBlacklistedAvoidArea(AvoidArea::CENTER_CIRCLE);

    auto intent_ptr = tactic.getNextIntent();
    ASSERT_TRUE(intent_ptr);
    EXPECT_EQ(
        std::vector<AvoidArea>({AvoidArea::FRIENDLY_HALF, AvoidArea::CENTER_CIRCLE}),
        intent_ptr->getAreasToAvoid());

    tactic.removeBlacklistedAvoidArea(AvoidArea::FRIENDLY_HALF);

    intent_ptr = tactic.getNextIntent();
    ASSERT_TRUE(intent_ptr);
    EXPECT_EQ(std::vector<AvoidArea>({AvoidArea::CENTER_CIRCLE}),
              intent_ptr->getAreasToAvoid());
}

TEST(TacticTest, extra_avoid_areas_overrides_whitelist)
{
    // Test adding and removing extra avoid areas is correctly reflected in the
    // avoid areas produced by the tactic
    Robot robot = Robot(0, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveTestTactic tactic = MoveTestTactic(true);
    tactic.updateRobot(robot);

    tactic.addWhitelistedAvoidArea(AvoidArea::FRIENDLY_HALF);

    tactic.addBlacklistedAvoidArea(AvoidArea::FRIENDLY_HALF);
    tactic.addBlacklistedAvoidArea(AvoidArea::CENTER_CIRCLE);

    auto intent_ptr = tactic.getNextIntent();
    ASSERT_TRUE(intent_ptr);
    EXPECT_EQ(
        std::vector<AvoidArea>({AvoidArea::FRIENDLY_HALF, AvoidArea::CENTER_CIRCLE}),
        intent_ptr->getAreasToAvoid());
}

TEST(TacticTest, test_whitelisted_areas_are_ignored)
{
    Robot robot = Robot(0, Point(1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveTestTactic tactic = MoveTestTactic(true);
    tactic.updateRobot(robot);

    World world;
    world.mutableGameState().updateRefboxGameState(RefboxGameState::DIRECT_FREE_THEM);

    tactic.addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);

    auto next_intent = tactic.getNextIntent(world);
    ASSERT_TRUE(next_intent);

    EXPECT_EQ(std::vector<AvoidArea>(
                  {AvoidArea::ENEMY_DEFENSE_AREA, AvoidArea::FRIENDLY_DEFENSE_AREA}),
              next_intent->getAreasToAvoid());
}

#include "software/ai/motion_constraint/motion_constraint_set_builder.h"

#include <gtest/gtest.h>

#include <set>

#include "software/ai/hl/stp/tactic/all_tactics.h"
#include "software/test_util/test_util.h"
#include "software/world/game_state.h"

// This namespace contains all the test parameters
namespace
{
    World world = ::TestUtil::createBlankTestingWorld();
    Pass pass({1, 1}, {0.5, 0}, 2.29);
    TbotsProto::AiConfig ai_config;

    // vector of pairs of Tactic and allowed MotionConstraints
    std::vector<
        std::pair<std::shared_ptr<Tactic>, std::set<TbotsProto::MotionConstraint>>>
        test_vector = {
            std::pair<std::shared_ptr<Tactic>, std::set<TbotsProto::MotionConstraint>>(
                new MoveTactic(), std::set<TbotsProto::MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<TbotsProto::MotionConstraint>>(
                new CreaseDefenderTactic(ai_config.robot_navigation_obstacle_config()),
                std::set<TbotsProto::MotionConstraint>(
                    {TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL})),
            std::pair<std::shared_ptr<Tactic>, std::set<TbotsProto::MotionConstraint>>(
                new GoalieTactic(ai_config),
                std::set<TbotsProto::MotionConstraint>(
                    {TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA,
                     TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA,
                     TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL,
                     TbotsProto::MotionConstraint::FRIENDLY_HALF})),
            std::pair<std::shared_ptr<Tactic>, std::set<TbotsProto::MotionConstraint>>(
                new ChipTactic(), std::set<TbotsProto::MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<TbotsProto::MotionConstraint>>(
                new KickoffChipTactic(),
                std::set<TbotsProto::MotionConstraint>(
                    {TbotsProto::MotionConstraint::CENTER_CIRCLE,
                     TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL})),
            std::pair<std::shared_ptr<Tactic>, std::set<TbotsProto::MotionConstraint>>(
                new PenaltyKickTactic(ai_config),
                std::set<TbotsProto::MotionConstraint>(
                    {TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL,
                     TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA,
                     TbotsProto::MotionConstraint::ENEMY_HALF})),
            std::pair<std::shared_ptr<Tactic>, std::set<TbotsProto::MotionConstraint>>(
                new PenaltySetupTactic(),
                std::set<TbotsProto::MotionConstraint>(
                    {TbotsProto::MotionConstraint::ENEMY_HALF,
                     TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA,
                     TbotsProto::MotionConstraint::FRIENDLY_HALF,
                     TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL})),
            std::pair<std::shared_ptr<Tactic>, std::set<TbotsProto::MotionConstraint>>(
                new ReceiverTactic(), std::set<TbotsProto::MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<TbotsProto::MotionConstraint>>(
                new ShadowEnemyTactic(), std::set<TbotsProto::MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<TbotsProto::MotionConstraint>>(
                new AttackerTactic(ai_config),
                std::set<TbotsProto::MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<TbotsProto::MotionConstraint>>(
                new StopTactic(), std::set<TbotsProto::MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<TbotsProto::MotionConstraint>>(
                new PassDefenderTactic(), std::set<TbotsProto::MotionConstraint>({}))};

    // sets of motion constraints for each type of game state
    auto stoppage_or_them_motion_constraints = std::set<TbotsProto::MotionConstraint>(
        {TbotsProto::MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA,
         TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL,
         TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA});

    auto gamestart_or_us_motion_constraints = std::set<TbotsProto::MotionConstraint>(
        {TbotsProto::MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA,
         TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA});

    auto kickoff_motion_constraints = std::set<TbotsProto::MotionConstraint>(
        {TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA,
         TbotsProto::MotionConstraint::CENTER_CIRCLE,
         TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL,
         TbotsProto::MotionConstraint::ENEMY_HALF});

    auto our_penalty_motion_constraints = std::set<TbotsProto::MotionConstraint>(
        {TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA,
         TbotsProto::MotionConstraint::ENEMY_HALF});

    auto them_penalty_motion_constraints = std::set<TbotsProto::MotionConstraint>(
        {TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA,
         TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL,
         TbotsProto::MotionConstraint::FRIENDLY_HALF});

    auto them_ball_placement = std::set<TbotsProto::MotionConstraint>(
        {TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL,
         TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA,
         TbotsProto::MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE});
}  // namespace

class CheckMotionConstraints
    : public ::testing::TestWithParam<
          std::pair<std::shared_ptr<Tactic>, std::set<TbotsProto::MotionConstraint>>>
{
   public:
    std::set<TbotsProto::MotionConstraint> correct_motion_constraints;
    GameState game_state;
};


TEST_P(CheckMotionConstraints, CycleStoppageOrThemGameStatesTest)
{
    correct_motion_constraints = stoppage_or_them_motion_constraints;

    for (TbotsProto::MotionConstraint c : GetParam().second)
    {
        correct_motion_constraints.erase(c);
    }

    game_state.updateRefereeCommand(RefereeCommand::HALT);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));

    game_state.updateRefereeCommand(RefereeCommand::STOP);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));

    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));

    game_state.updateRefereeCommand(RefereeCommand::DIRECT_FREE_THEM);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));

    game_state.updateRefereeCommand(RefereeCommand::INDIRECT_FREE_THEM);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));

    game_state.updateRefereeCommand(RefereeCommand::TIMEOUT_US);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));

    game_state.updateRefereeCommand(RefereeCommand::TIMEOUT_THEM);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));

    game_state.updateRefereeCommand(RefereeCommand::GOAL_US);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));

    game_state.updateRefereeCommand(RefereeCommand::GOAL_THEM);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));
}

TEST_P(CheckMotionConstraints, CycleGameStartOrUsGameStatesTest)
{
    correct_motion_constraints = gamestart_or_us_motion_constraints;

    for (TbotsProto::MotionConstraint c : GetParam().second)
    {
        correct_motion_constraints.erase(c);
    }

    game_state.updateRefereeCommand(RefereeCommand::FORCE_START);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));

    game_state.updateRefereeCommand(RefereeCommand::DIRECT_FREE_US);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));

    game_state.updateRefereeCommand(RefereeCommand::INDIRECT_FREE_US);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));

    game_state.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_US);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));
}

TEST_P(CheckMotionConstraints, CycleKickoffGameStatesTest)
{
    correct_motion_constraints = kickoff_motion_constraints;

    for (TbotsProto::MotionConstraint c : GetParam().second)
    {
        correct_motion_constraints.erase(c);
    }

    game_state.updateRefereeCommand(RefereeCommand::PREPARE_KICKOFF_US);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));

    game_state.updateRefereeCommand(RefereeCommand::PREPARE_KICKOFF_THEM);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));
}

TEST_P(CheckMotionConstraints, CycleOurPenaltyGameStatesTest)
{
    correct_motion_constraints = our_penalty_motion_constraints;

    for (TbotsProto::MotionConstraint c : GetParam().second)
    {
        correct_motion_constraints.erase(c);
    }

    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));
}

TEST_P(CheckMotionConstraints, CycleThemPenaltyGameStatesTest)
{
    correct_motion_constraints = them_penalty_motion_constraints;

    for (TbotsProto::MotionConstraint c : GetParam().second)
    {
        correct_motion_constraints.erase(c);
    }

    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_THEM);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));
}

TEST_P(CheckMotionConstraints, CycleThemBallPlacementTest)
{
    correct_motion_constraints = them_ball_placement;

    for (TbotsProto::MotionConstraint c : GetParam().second)
    {
        correct_motion_constraints.erase(c);
    }

    game_state.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_THEM);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));
}


INSTANTIATE_TEST_CASE_P(CycleStoppageOrThemGameStatesTest, CheckMotionConstraints,
                        ::testing::ValuesIn(test_vector.begin(), test_vector.end()));

INSTANTIATE_TEST_CASE_P(CycleGameStartOrUsGameStatesTest, CheckMotionConstraints,
                        ::testing::ValuesIn(test_vector.begin(), test_vector.end()));

INSTANTIATE_TEST_CASE_P(CycleKickoffGameStatesTest, CheckMotionConstraints,
                        ::testing::ValuesIn(test_vector.begin(), test_vector.end()));

INSTANTIATE_TEST_CASE_P(CycleOurPenaltyGameStatesTest, CheckMotionConstraints,
                        ::testing::ValuesIn(test_vector.begin(), test_vector.end()));

INSTANTIATE_TEST_CASE_P(CycleThemPenaltyGameStatesTest, CheckMotionConstraints,
                        ::testing::ValuesIn(test_vector.begin(), test_vector.end()));

INSTANTIATE_TEST_CASE_P(CheckMotionConstraints_CycleThemBallPlacementTest_Test,
                        CheckMotionConstraints,
                        ::testing::ValuesIn(test_vector.begin(), test_vector.end()));

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


    // vector of pairs of Tactic and allowed MotionConstraints
    std::vector<std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>>
        test_vector = {
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new MoveTactic(false), std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new CreaseDefenderTactic(
                    std::make_shared<const RobotNavigationObstacleConfig>()),
                std::set<MotionConstraint>({MotionConstraint::HALF_METER_AROUND_BALL})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new GoalieTactic(std::make_shared<const GoalieTacticConfig>()),
                std::set<MotionConstraint>({MotionConstraint::FRIENDLY_DEFENSE_AREA,
                                            MotionConstraint::FRIENDLY_DEFENSE_AREA,
                                            MotionConstraint::HALF_METER_AROUND_BALL,
                                            MotionConstraint::FRIENDLY_HALF})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new ChipTactic(true), std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new KickoffChipTactic(true),
                std::set<MotionConstraint>({MotionConstraint::CENTER_CIRCLE,
                                            MotionConstraint::HALF_METER_AROUND_BALL})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new PenaltyKickTactic(),
                std::set<MotionConstraint>({MotionConstraint::HALF_METER_AROUND_BALL,
                                            MotionConstraint::ENEMY_DEFENSE_AREA,
                                            MotionConstraint::ENEMY_HALF})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new PenaltySetupTactic(true),
                std::set<MotionConstraint>({MotionConstraint::ENEMY_HALF,
                                            MotionConstraint::ENEMY_DEFENSE_AREA,
                                            MotionConstraint::FRIENDLY_HALF,
                                            MotionConstraint::HALF_METER_AROUND_BALL})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new ReceiverTactic(pass), std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new ShadowEnemyTactic(), std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new ShadowFreekickerTactic(ShadowFreekickerTactic::LEFT,
                                           world.enemyTeam(), world.ball(), world.field(),
                                           false),
                std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new AttackerTactic(std::make_shared<const AttackerTacticConfig>()),
                std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new StopTactic(false), std::set<MotionConstraint>({}))};

    // sets of motion constraints for each type of game state
    auto stoppage_or_them_motion_constraints =
        std::set<MotionConstraint>({MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA,
                                    MotionConstraint::ENEMY_ROBOTS_COLLISION,
                                    MotionConstraint::HALF_METER_AROUND_BALL,
                                    MotionConstraint::FRIENDLY_DEFENSE_AREA});

    auto gamestart_or_us_motion_constraints =
        std::set<MotionConstraint>({MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA,
                                    MotionConstraint::ENEMY_ROBOTS_COLLISION,
                                    MotionConstraint::FRIENDLY_DEFENSE_AREA});

    auto kickoff_motion_constraints = std::set<MotionConstraint>(
        {MotionConstraint::FRIENDLY_DEFENSE_AREA, MotionConstraint::CENTER_CIRCLE,
         MotionConstraint::HALF_METER_AROUND_BALL, MotionConstraint::ENEMY_HALF,
         MotionConstraint::ENEMY_ROBOTS_COLLISION});

    auto our_penalty_motion_constraints = std::set<MotionConstraint>(
        {MotionConstraint::FRIENDLY_DEFENSE_AREA, MotionConstraint::ENEMY_HALF,
         MotionConstraint::ENEMY_ROBOTS_COLLISION});

    auto them_penalty_motion_constraints = std::set<MotionConstraint>(
        {MotionConstraint::FRIENDLY_DEFENSE_AREA,
         MotionConstraint::HALF_METER_AROUND_BALL, MotionConstraint::FRIENDLY_HALF,
         MotionConstraint::ENEMY_ROBOTS_COLLISION});
}  // namespace

class CheckMotionConstraints
    : public ::testing::TestWithParam<
          std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>>
{
   public:
    std::set<MotionConstraint> correct_motion_constraints;
    GameState game_state;
};


TEST_P(CheckMotionConstraints, CycleStoppageOrThemGameStatesTest)
{
    correct_motion_constraints = stoppage_or_them_motion_constraints;

    for (MotionConstraint c : GetParam().second)
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

    game_state.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_THEM);
    EXPECT_EQ(correct_motion_constraints,
              buildMotionConstraintSet(game_state, *GetParam().first));
}

TEST_P(CheckMotionConstraints, CycleGameStartOrUsGameStatesTest)
{
    correct_motion_constraints = gamestart_or_us_motion_constraints;

    for (MotionConstraint c : GetParam().second)
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

    for (MotionConstraint c : GetParam().second)
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

    for (MotionConstraint c : GetParam().second)
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

    for (MotionConstraint c : GetParam().second)
    {
        correct_motion_constraints.erase(c);
    }

    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_THEM);
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

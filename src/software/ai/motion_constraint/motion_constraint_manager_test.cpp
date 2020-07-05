/**
 * This file contains the unit tests for functions in motion_constraint_manager.cpp
 */

#include "software/ai/motion_constraint/motion_constraint_manager.h"

#include <gtest/gtest.h>

#include <set>

#include "software/ai/hl/stp/tactic/all_tactics.h"
#include "software/test_util/test_util.h"
#include "software/world/game_state.h"

// This namespace contains all the test parameters
namespace
{
    World world = ::TestUtil::createBlankTestingWorld();
    Pass pass({1, 1}, {0.5, 0}, 2.29, Timestamp::fromSeconds(5));


    // vector of pairs of Tactic and whitelists
    std::vector<std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>>
        test_vector = {
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new MoveTactic(false), std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new CherryPickTactic(world, Rectangle({0, 0}, {1, 1})),
                std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new CreaseDefenderTactic(world.field(), world.ball(),
                                         world.friendlyTeam(), world.enemyTeam(),
                                         CreaseDefenderTactic::LEFT),
                std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new GoalieTactic(world.ball(), world.field(), world.friendlyTeam(),
                                 world.enemyTeam()),
                std::set<MotionConstraint>({MotionConstraint::FRIENDLY_DEFENSE_AREA,
                                            MotionConstraint::FRIENDLY_DEFENSE_AREA,
                                            MotionConstraint::HALF_METER_AROUND_BALL,
                                            MotionConstraint::FRIENDLY_HALF})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new ChipTactic(world.ball(), true), std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new KickoffChipTactic(world.ball(), true),
                std::set<MotionConstraint>({MotionConstraint::CENTER_CIRCLE,
                                            MotionConstraint::HALF_METER_AROUND_BALL})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new PenaltyKickTactic(world.ball(), world.field(), std::nullopt, true),
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
                new PatrolTactic(std::vector<Point>(), 0.0, Angle::zero(), 0.0),
                std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new ReceiverTactic(world.field(), world.friendlyTeam(), world.enemyTeam(),
                                   pass, world.ball(), false),
                std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new ShadowEnemyTactic(world.field(), world.friendlyTeam(),
                                      world.enemyTeam(), true, world.ball(), 0.5, false,
                                      true),
                std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new ShadowFreekickerTactic(ShadowFreekickerTactic::LEFT,
                                           world.enemyTeam(), world.ball(), world.field(),
                                           false),
                std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new ShootGoalTactic(world.field(), world.friendlyTeam(),
                                    world.enemyTeam(), world.ball(), Angle::zero(),
                                    std::nullopt, false),
                std::set<MotionConstraint>({MotionConstraint::HALF_METER_AROUND_BALL})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new PasserTactic(pass, world.ball(), false),
                std::set<MotionConstraint>({})),
            std::pair<std::shared_ptr<Tactic>, std::set<MotionConstraint>>(
                new StopTactic(false), std::set<MotionConstraint>({}))};

    // sets of motion constraints for each type of gamestate
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
    MotionConstraintManager manager;
};


TEST_P(CheckMotionConstraints, CycleStoppageOrThemGameStatesTest)
{
    correct_motion_constraints = stoppage_or_them_motion_constraints;

    for (MotionConstraint c : GetParam().second)
    {
        correct_motion_constraints.erase(c);
    }

    game_state.updateRefboxGameState(RefboxGameState::HALT);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));

    game_state.updateRefboxGameState(RefboxGameState::STOP);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));

    game_state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));

    game_state.updateRefboxGameState(RefboxGameState::DIRECT_FREE_THEM);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));

    game_state.updateRefboxGameState(RefboxGameState::INDIRECT_FREE_THEM);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));

    game_state.updateRefboxGameState(RefboxGameState::TIMEOUT_US);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));

    game_state.updateRefboxGameState(RefboxGameState::TIMEOUT_THEM);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));

    game_state.updateRefboxGameState(RefboxGameState::GOAL_US);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));

    game_state.updateRefboxGameState(RefboxGameState::GOAL_THEM);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));

    game_state.updateRefboxGameState(RefboxGameState::BALL_PLACEMENT_THEM);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));
}

TEST_P(CheckMotionConstraints, CycleGameStartOrUsGameStatesTest)
{
    correct_motion_constraints = gamestart_or_us_motion_constraints;

    for (MotionConstraint c : GetParam().second)
    {
        correct_motion_constraints.erase(c);
    }

    game_state.updateRefboxGameState(RefboxGameState::FORCE_START);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));

    game_state.updateRefboxGameState(RefboxGameState::DIRECT_FREE_US);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));

    game_state.updateRefboxGameState(RefboxGameState::INDIRECT_FREE_US);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));

    game_state.updateRefboxGameState(RefboxGameState::BALL_PLACEMENT_US);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));
}

TEST_P(CheckMotionConstraints, CycleKickoffGameStatesTest)
{
    correct_motion_constraints = kickoff_motion_constraints;

    for (MotionConstraint c : GetParam().second)
    {
        correct_motion_constraints.erase(c);
    }

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_KICKOFF_US);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_KICKOFF_THEM);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));
}

TEST_P(CheckMotionConstraints, CycleOurPenaltyGameStatesTest)
{
    correct_motion_constraints = our_penalty_motion_constraints;

    for (MotionConstraint c : GetParam().second)
    {
        correct_motion_constraints.erase(c);
    }

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_PENALTY_US);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));
}

TEST_P(CheckMotionConstraints, CycleThemPenaltyGameStatesTest)
{
    correct_motion_constraints = them_penalty_motion_constraints;

    for (MotionConstraint c : GetParam().second)
    {
        correct_motion_constraints.erase(c);
    }

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_PENALTY_THEM);
    EXPECT_EQ(correct_motion_constraints,
              manager.getMotionConstraints(game_state, *GetParam().first));
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

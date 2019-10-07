/**
 * This file contains the unit tests for functions in motion_constraint_manager.cpp
 */

#include "software/ai/motion_constraint/motion_constraint_manager.h"

#include <gtest/gtest.h>

#include <set>

#include "software/ai/hl/stp/tactic/all_tactics.h"
#include "software/ai/world/game_state.h"

TEST(MotionConstraintTest, test_motion_constraints_from_game_stoppage_or_them_states)
{
    GameState game_state;
    MoveTactic move_tactic;
    MotionConstraintManager manager;
    std::set<MotionConstraint> correct_motion_constraints;

    game_state.updateRefboxGameState(RefboxGameState::HALT);
    correct_motion_constraints = std::set<MotionConstraint>(
        {MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, MotionConstraint::ENEMY_ROBOTS_COLLISION,
         MotionConstraint::HALF_METER_AROUND_BALL, MotionConstraint::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::STOP);
    correct_motion_constraints = std::set<MotionConstraint>(
        {MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, MotionConstraint::ENEMY_ROBOTS_COLLISION,
         MotionConstraint::HALF_METER_AROUND_BALL, MotionConstraint::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    correct_motion_constraints = std::set<MotionConstraint>(
        {MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, MotionConstraint::ENEMY_ROBOTS_COLLISION,
         MotionConstraint::HALF_METER_AROUND_BALL, MotionConstraint::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::DIRECT_FREE_THEM);
    correct_motion_constraints = std::set<MotionConstraint>(
        {MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, MotionConstraint::ENEMY_ROBOTS_COLLISION,
         MotionConstraint::HALF_METER_AROUND_BALL, MotionConstraint::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::INDIRECT_FREE_THEM);
    correct_motion_constraints = std::set<MotionConstraint>(
        {MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, MotionConstraint::ENEMY_ROBOTS_COLLISION,
         MotionConstraint::HALF_METER_AROUND_BALL, MotionConstraint::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::TIMEOUT_US);
    correct_motion_constraints = std::set<MotionConstraint>(
        {MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, MotionConstraint::ENEMY_ROBOTS_COLLISION,
         MotionConstraint::HALF_METER_AROUND_BALL, MotionConstraint::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::TIMEOUT_THEM);
    correct_motion_constraints = std::set<MotionConstraint>(
        {MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, MotionConstraint::ENEMY_ROBOTS_COLLISION,
         MotionConstraint::HALF_METER_AROUND_BALL, MotionConstraint::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::GOAL_US);
    correct_motion_constraints = std::set<MotionConstraint>(
        {MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, MotionConstraint::ENEMY_ROBOTS_COLLISION,
         MotionConstraint::HALF_METER_AROUND_BALL, MotionConstraint::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::GOAL_THEM);
    correct_motion_constraints = std::set<MotionConstraint>(
        {MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, MotionConstraint::ENEMY_ROBOTS_COLLISION,
         MotionConstraint::HALF_METER_AROUND_BALL, MotionConstraint::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::BALL_PLACEMENT_THEM);
    correct_motion_constraints = std::set<MotionConstraint>(
        {MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, MotionConstraint::ENEMY_ROBOTS_COLLISION,
         MotionConstraint::HALF_METER_AROUND_BALL, MotionConstraint::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));
}

TEST(MotionConstraintTest, test_motion_constraints_from_game_start_or_us_states)
{
    GameState game_state;
    MoveTactic move_tactic;
    MotionConstraintManager manager;
    std::set<MotionConstraint> correct_motion_constraints;

    game_state.updateRefboxGameState(RefboxGameState::FORCE_START);
    correct_motion_constraints = std::set<MotionConstraint>({MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA,
                                             MotionConstraint::ENEMY_ROBOTS_COLLISION,
                                             MotionConstraint::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::DIRECT_FREE_US);
    correct_motion_constraints = std::set<MotionConstraint>({MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA,
                                             MotionConstraint::ENEMY_ROBOTS_COLLISION,
                                             MotionConstraint::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::INDIRECT_FREE_US);
    correct_motion_constraints = std::set<MotionConstraint>({MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA,
                                             MotionConstraint::ENEMY_ROBOTS_COLLISION,
                                             MotionConstraint::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::BALL_PLACEMENT_US);
    correct_motion_constraints = std::set<MotionConstraint>({MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA,
                                             MotionConstraint::ENEMY_ROBOTS_COLLISION,
                                             MotionConstraint::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));
}

TEST(MotionConstraintTest, test_motion_constraints_from_kickoff_states)
{
    GameState game_state;
    MoveTactic move_tactic;
    MotionConstraintManager manager;
    std::set<MotionConstraint> correct_motion_constraints;

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_KICKOFF_US);
    correct_motion_constraints =
        std::set<MotionConstraint>({MotionConstraint::FRIENDLY_DEFENSE_AREA, MotionConstraint::CENTER_CIRCLE,
                            MotionConstraint::HALF_METER_AROUND_BALL, MotionConstraint::ENEMY_HALF,
                            MotionConstraint::ENEMY_ROBOTS_COLLISION});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_KICKOFF_THEM);
    correct_motion_constraints =
        std::set<MotionConstraint>({MotionConstraint::FRIENDLY_DEFENSE_AREA, MotionConstraint::CENTER_CIRCLE,
                            MotionConstraint::HALF_METER_AROUND_BALL, MotionConstraint::ENEMY_HALF,
                            MotionConstraint::ENEMY_ROBOTS_COLLISION});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));
}

TEST(MotionConstraintTest, test_motion_constraints_from_our_penalty_state)
{
    GameState game_state;
    MoveTactic move_tactic;
    MotionConstraintManager manager;
    std::set<MotionConstraint> correct_motion_constraints;

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_PENALTY_US);
    correct_motion_constraints =
        std::set<MotionConstraint>({MotionConstraint::FRIENDLY_DEFENSE_AREA, MotionConstraint::ENEMY_HALF,
                            MotionConstraint::ENEMY_ROBOTS_COLLISION});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));
}

TEST(MotionConstraintTest, test_motion_constraints_from_them_penalty_state)
{
    GameState game_state;
    MoveTactic move_tactic;
    MotionConstraintManager manager;
    std::set<MotionConstraint> correct_motion_constraints;

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_PENALTY_THEM);
    correct_motion_constraints = std::set<MotionConstraint>(
        {MotionConstraint::FRIENDLY_DEFENSE_AREA, MotionConstraint::HALF_METER_AROUND_BALL,
         MotionConstraint::FRIENDLY_HALF, MotionConstraint::ENEMY_ROBOTS_COLLISION});
    EXPECT_EQ(correct_motion_constraints, manager.getMotionConstraints(game_state, move_tactic));
}

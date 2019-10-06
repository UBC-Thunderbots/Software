/**
 * This file contains the unit tests for functions in move_rule_manager.cpp
 */

#include "software/ai/move_rule/move_rule_manager.h"

#include <gtest/gtest.h>

#include <set>

#include "software/ai/hl/stp/tactic/all_tactics.h"
#include "software/ai/world/game_state.h"

TEST(MoveRuleTest, test_avoid_areas_from_game_stoppage_or_them_states)
{
    GameState game_state;
    MoveTactic move_tactic;
    MoveRuleManager manager;
    std::set<MoveRule> correct_move_rules;

    game_state.updateRefboxGameState(RefboxGameState::HALT);
    correct_move_rules = std::set<MoveRule>(
        {MoveRule::INFLATED_ENEMY_DEFENSE_AREA, MoveRule::ENEMY_ROBOTS_COLLISION,
         MoveRule::HALF_METER_AROUND_BALL, MoveRule::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::STOP);
    correct_move_rules = std::set<MoveRule>(
        {MoveRule::INFLATED_ENEMY_DEFENSE_AREA, MoveRule::ENEMY_ROBOTS_COLLISION,
         MoveRule::HALF_METER_AROUND_BALL, MoveRule::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    correct_move_rules = std::set<MoveRule>(
        {MoveRule::INFLATED_ENEMY_DEFENSE_AREA, MoveRule::ENEMY_ROBOTS_COLLISION,
         MoveRule::HALF_METER_AROUND_BALL, MoveRule::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::DIRECT_FREE_THEM);
    correct_move_rules = std::set<MoveRule>(
        {MoveRule::INFLATED_ENEMY_DEFENSE_AREA, MoveRule::ENEMY_ROBOTS_COLLISION,
         MoveRule::HALF_METER_AROUND_BALL, MoveRule::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::INDIRECT_FREE_THEM);
    correct_move_rules = std::set<MoveRule>(
        {MoveRule::INFLATED_ENEMY_DEFENSE_AREA, MoveRule::ENEMY_ROBOTS_COLLISION,
         MoveRule::HALF_METER_AROUND_BALL, MoveRule::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::TIMEOUT_US);
    correct_move_rules = std::set<MoveRule>(
        {MoveRule::INFLATED_ENEMY_DEFENSE_AREA, MoveRule::ENEMY_ROBOTS_COLLISION,
         MoveRule::HALF_METER_AROUND_BALL, MoveRule::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::TIMEOUT_THEM);
    correct_move_rules = std::set<MoveRule>(
        {MoveRule::INFLATED_ENEMY_DEFENSE_AREA, MoveRule::ENEMY_ROBOTS_COLLISION,
         MoveRule::HALF_METER_AROUND_BALL, MoveRule::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::GOAL_US);
    correct_move_rules = std::set<MoveRule>(
        {MoveRule::INFLATED_ENEMY_DEFENSE_AREA, MoveRule::ENEMY_ROBOTS_COLLISION,
         MoveRule::HALF_METER_AROUND_BALL, MoveRule::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::GOAL_THEM);
    correct_move_rules = std::set<MoveRule>(
        {MoveRule::INFLATED_ENEMY_DEFENSE_AREA, MoveRule::ENEMY_ROBOTS_COLLISION,
         MoveRule::HALF_METER_AROUND_BALL, MoveRule::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::BALL_PLACEMENT_THEM);
    correct_move_rules = std::set<MoveRule>(
        {MoveRule::INFLATED_ENEMY_DEFENSE_AREA, MoveRule::ENEMY_ROBOTS_COLLISION,
         MoveRule::HALF_METER_AROUND_BALL, MoveRule::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));
}

TEST(MoveRuleTest, test_avoid_areas_from_game_start_or_us_states)
{
    GameState game_state;
    MoveTactic move_tactic;
    MoveRuleManager manager;
    std::set<MoveRule> correct_move_rules;

    game_state.updateRefboxGameState(RefboxGameState::FORCE_START);
    correct_move_rules = std::set<MoveRule>({MoveRule::INFLATED_ENEMY_DEFENSE_AREA,
                                             MoveRule::ENEMY_ROBOTS_COLLISION,
                                             MoveRule::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::DIRECT_FREE_US);
    correct_move_rules = std::set<MoveRule>({MoveRule::INFLATED_ENEMY_DEFENSE_AREA,
                                             MoveRule::ENEMY_ROBOTS_COLLISION,
                                             MoveRule::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::INDIRECT_FREE_US);
    correct_move_rules = std::set<MoveRule>({MoveRule::INFLATED_ENEMY_DEFENSE_AREA,
                                             MoveRule::ENEMY_ROBOTS_COLLISION,
                                             MoveRule::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::BALL_PLACEMENT_US);
    correct_move_rules = std::set<MoveRule>({MoveRule::INFLATED_ENEMY_DEFENSE_AREA,
                                             MoveRule::ENEMY_ROBOTS_COLLISION,
                                             MoveRule::FRIENDLY_DEFENSE_AREA});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));
}

TEST(MoveRuleTest, test_avoid_areas_from_kickoff_states)
{
    GameState game_state;
    MoveTactic move_tactic;
    MoveRuleManager manager;
    std::set<MoveRule> correct_move_rules;

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_KICKOFF_US);
    correct_move_rules =
        std::set<MoveRule>({MoveRule::FRIENDLY_DEFENSE_AREA, MoveRule::CENTER_CIRCLE,
                            MoveRule::HALF_METER_AROUND_BALL, MoveRule::ENEMY_HALF,
                            MoveRule::ENEMY_ROBOTS_COLLISION});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_KICKOFF_THEM);
    correct_move_rules =
        std::set<MoveRule>({MoveRule::FRIENDLY_DEFENSE_AREA, MoveRule::CENTER_CIRCLE,
                            MoveRule::HALF_METER_AROUND_BALL, MoveRule::ENEMY_HALF,
                            MoveRule::ENEMY_ROBOTS_COLLISION});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));
}

TEST(MoveRuleTest, test_avoid_areas_from_our_penalty_state)
{
    GameState game_state;
    MoveTactic move_tactic;
    MoveRuleManager manager;
    std::set<MoveRule> correct_move_rules;

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_PENALTY_US);
    correct_move_rules =
        std::set<MoveRule>({MoveRule::FRIENDLY_DEFENSE_AREA, MoveRule::ENEMY_HALF,
                            MoveRule::ENEMY_ROBOTS_COLLISION});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));
}

TEST(MoveRuleTest, test_avoid_areas_from_them_penalty_state)
{
    GameState game_state;
    MoveTactic move_tactic;
    MoveRuleManager manager;
    std::set<MoveRule> correct_move_rules;

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_PENALTY_THEM);
    correct_move_rules = std::set<MoveRule>(
        {MoveRule::FRIENDLY_DEFENSE_AREA, MoveRule::HALF_METER_AROUND_BALL,
         MoveRule::FRIENDLY_HALF, MoveRule::ENEMY_ROBOTS_COLLISION});
    EXPECT_EQ(correct_move_rules, manager.getMoveRules(game_state, move_tactic));
}

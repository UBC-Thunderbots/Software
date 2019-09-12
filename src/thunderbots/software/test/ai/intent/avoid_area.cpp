/**
 * This file contains the unit tests for functions in avoid_area.cpp
 */

#include "ai/intent/avoid_area.h"

#include <gtest/gtest.h>

TEST(AvoidAreaTest, test_avoid_areas_from_game_stoppage_or_them_states)
{
    GameState game_state;

    game_state.updateRefboxGameState(RefboxGameState::HALT);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS |
                              1 << (uint32_t)AvoidArea::HALF_METER_AROUND_BALL |
                              1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA),
              getAvoidAreasFromGameState(game_state));
    game_state.updateRefboxGameState(RefboxGameState::STOP);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS |
                              1 << (uint32_t)AvoidArea::HALF_METER_AROUND_BALL |
                              1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA),
              getAvoidAreasFromGameState(game_state));
    game_state.updateRefboxGameState(RefboxGameState::NORMAL_START);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS |
                              1 << (uint32_t)AvoidArea::HALF_METER_AROUND_BALL |
                              1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA),
              getAvoidAreasFromGameState(game_state));
    game_state.updateRefboxGameState(RefboxGameState::DIRECT_FREE_THEM);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS |
                              1 << (uint32_t)AvoidArea::HALF_METER_AROUND_BALL |
                              1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA),
              getAvoidAreasFromGameState(game_state));
    game_state.updateRefboxGameState(RefboxGameState::INDIRECT_FREE_THEM);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS |
                              1 << (uint32_t)AvoidArea::HALF_METER_AROUND_BALL |
                              1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA),
              getAvoidAreasFromGameState(game_state));
    game_state.updateRefboxGameState(RefboxGameState::TIMEOUT_US);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS |
                              1 << (uint32_t)AvoidArea::HALF_METER_AROUND_BALL |
                              1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA),
              getAvoidAreasFromGameState(game_state));
    game_state.updateRefboxGameState(RefboxGameState::TIMEOUT_THEM);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS |
                              1 << (uint32_t)AvoidArea::HALF_METER_AROUND_BALL |
                              1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA),
              getAvoidAreasFromGameState(game_state));
    game_state.updateRefboxGameState(RefboxGameState::GOAL_US);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS |
                              1 << (uint32_t)AvoidArea::HALF_METER_AROUND_BALL |
                              1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA),
              getAvoidAreasFromGameState(game_state));
    game_state.updateRefboxGameState(RefboxGameState::GOAL_THEM);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS |
                              1 << (uint32_t)AvoidArea::HALF_METER_AROUND_BALL |
                              1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA),
              getAvoidAreasFromGameState(game_state));
    game_state.updateRefboxGameState(RefboxGameState::BALL_PLACEMENT_THEM);

    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS |
                              1 << (uint32_t)AvoidArea::HALF_METER_AROUND_BALL |
                              1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA),
              getAvoidAreasFromGameState(game_state));
}

TEST(AvoidAreaTest, test_avoid_areas_from_game_start_or_us_states)
{
    GameState game_state;

    game_state.updateRefboxGameState(RefboxGameState::FORCE_START);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS |
                              1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA),
              getAvoidAreasFromGameState(game_state));
    game_state.updateRefboxGameState(RefboxGameState::DIRECT_FREE_US);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS |
                              1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA),
              getAvoidAreasFromGameState(game_state));
    game_state.updateRefboxGameState(RefboxGameState::INDIRECT_FREE_US);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS |
                              1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA),
              getAvoidAreasFromGameState(game_state));
    game_state.updateRefboxGameState(RefboxGameState::BALL_PLACEMENT_US);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS |
                              1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA),
              getAvoidAreasFromGameState(game_state));
}

TEST(AvoidAreaTest, test_avoid_areas_from_kickoff_states)
{
    GameState game_state;

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_KICKOFF_US);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::CENTER_CIRCLE |
                              1 << (uint32_t)AvoidArea::HALF_METER_AROUND_BALL |
                              1 << (uint32_t)AvoidArea::ENEMY_HALF |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS),
              getAvoidAreasFromGameState(game_state));
    game_state.updateRefboxGameState(RefboxGameState::PREPARE_KICKOFF_THEM);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::CENTER_CIRCLE |
                              1 << (uint32_t)AvoidArea::HALF_METER_AROUND_BALL |
                              1 << (uint32_t)AvoidArea::ENEMY_HALF |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS),
              getAvoidAreasFromGameState(game_state));
}

TEST(AvoidAreaTest, test_avoid_areas_from_our_penalty_state)
{
    GameState game_state;

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_PENALTY_US);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::ENEMY_HALF |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS),
              getAvoidAreasFromGameState(game_state));
}

TEST(AvoidAreaTest, test_avoid_areas_from_them_penalty_state)
{
    GameState game_state;

    game_state.updateRefboxGameState(RefboxGameState::PREPARE_PENALTY_THEM);
    EXPECT_EQ(std::bitset<32>(1 << (uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA |
                              1 << (uint32_t)AvoidArea::HALF_METER_AROUND_BALL |
                              1 << (uint32_t)AvoidArea::FRIENDLY_HALF |
                              1 << (uint32_t)AvoidArea::ENEMY_ROBOTS),
              getAvoidAreasFromGameState(game_state));
}

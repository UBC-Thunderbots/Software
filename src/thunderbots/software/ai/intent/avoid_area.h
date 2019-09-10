#pragma once

#include <ostream>

#include "ai/world/world.h"

/**
 * This enum describes the areas that an intent can choose to avoid. They serve
 * as an indicator to the navigator when it interprets the intents
 */
enum class AvoidArea
{
    // Used for iterating over avoid areas
    FIRST_AVOID_AREA = 1 << 8,
    // The box around the friendly goal
    FRIENDLY_DEFENSE_AREA = 1 << 0,
    // The box around the enemy goal
    ENEMY_DEFENSE_AREA = 1 << 1,
    // The box around the enemy goal, inflated by 0.2 meters for certain situations
    INFLATED_ENEMY_DEFENSE_AREA = 1 << 2,
    // The center circle
    CENTER_CIRCLE = 1 << 3,
    // A half meter radius around the ball
    HALF_METER_AROUND_BALL = 1 << 4,
    // The ball itself
    BALL = 1 << 5,
    // The enemy half of the field
    ENEMY_HALF = 1 << 6,
    // The friendly half of the field
    FRIENDLY_HALF = 1 << 7,
    // Used for iterating over avoid areas
    LAST_AVOID_AREA_BEFORE_ROBOTS = 1 << 7,
    // The enemy robots and their avoid obstacles
    ENEMY_ROBOTS = 1 << 8,
    // Used for iterating over avoid areas
    LAST_AVOID_AREA = 1 << 8,
};

typedef uint32_t avoid_area_mask_t;

// Adds avoid areas together
#define ADD_AVOID_AREA(a, b) (avoid_area_mask_t) a | (avoid_area_mask_t)b
// sets a = a + b
#define MERGE_AVOID_AREAS(a, b) a |= (avoid_area_mask_t)b
// sets a = a - b
#define REMOVE_AVOID_AREAS(a, b) a &= (~((avoid_area_mask_t)b))

std::ostream& operator<<(std::ostream& os, const AvoidArea& state);

/**
 * Get all the areas that this tactic should not be allowed to move into based on
 * game state
 *
 * @param game_state The current game state
 *
 * @return avoid area mask this tactic should avoid
 */
avoid_area_mask_t getAvoidAreasFromGameState(const GameState& game_state);

#pragma once

#include <ostream>

/**
 * This enum describes the rules that robot movement must respect. They serve
 * as an indicator to the navigator when it interprets the intents
 */
enum class MoveRule
{
    // Avoid collision with enemy robots above a certain speed
    ENEMY_ROBOTS_COLLISION,
    // The box around the friendly goal
    FRIENDLY_DEFENSE_AREA,
    // The box around the enemy goal
    ENEMY_DEFENSE_AREA,
    // The box around the enemy goal, inflated by 0.2 meters for certain situations
    INFLATED_ENEMY_DEFENSE_AREA,
    // The center circle
    CENTER_CIRCLE,
    // A half meter radius around the ball
    HALF_METER_AROUND_BALL,
    // The enemy half of the field
    ENEMY_HALF,
    // The friendly half of the field
    FRIENDLY_HALF
};

std::ostream& operator<<(std::ostream& os, const MoveRule& state);

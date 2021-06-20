#pragma once

#include <ostream>

#include "software/util/make_enum/make_enum.h"

/**
 * This enum describes the constraints robot movement. They are
 * used for constraining navigation
 */
MAKE_ENUM(MotionConstraint,
          // The box around the friendly goal
          FRIENDLY_DEFENSE_AREA,
          // The box around the enemy goal
          ENEMY_DEFENSE_AREA,
          // The inflated box around the enemy defense area
          INFLATED_ENEMY_DEFENSE_AREA,
          // The center circle
          CENTER_CIRCLE,
          // A half meter radius around the ball
          HALF_METER_AROUND_BALL,
          // The enemy half of the field
          ENEMY_HALF,
          // The friendly half of the field
          FRIENDLY_HALF);

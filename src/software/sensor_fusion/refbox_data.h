#pragma once

#include "software/util/make_enum/make_enum.h"

// clang-format off
MAKE_ENUM(RefboxGameState,
          // these enum items map to the constants in RefboxCommand.msg
          HALT,
          STOP,
          NORMAL_START,
          FORCE_START,
          PREPARE_KICKOFF_US,
          PREPARE_KICKOFF_THEM,
          PREPARE_PENALTY_US,
          PREPARE_PENALTY_THEM,
          DIRECT_FREE_US,
          DIRECT_FREE_THEM,
          INDIRECT_FREE_US,
          INDIRECT_FREE_THEM,
          TIMEOUT_US,
          TIMEOUT_THEM,
          GOAL_US,
          GOAL_THEM,
          BALL_PLACEMENT_US,
          BALL_PLACEMENT_THEM,
          REFBOX_GAME_STATE_COUNT);
// clang-format on

MAKE_ENUM(RefboxStage,
          // The first half is about to start.
          // A kickoff is called within this stage.
          // This stage ends with the NORMAL_START.
          NORMAL_FIRST_HALF_PRE,
          // The first half of the normal game, before half time.
          NORMAL_FIRST_HALF,
          // Half time between first and second halves.
          NORMAL_HALF_TIME,
          // The second half is about to start.
          // A kickoff is called within this stage.
          // This stage ends with the NORMAL_START.
          NORMAL_SECOND_HALF_PRE,
          // The second half of the normal game, after half time.
          NORMAL_SECOND_HALF,
          // The break before extra time.
          EXTRA_TIME_BREAK,
          // The first half of extra time is about to start.
          // A kickoff is called within this stage.
          // This stage ends with the NORMAL_START.
          EXTRA_FIRST_HALF_PRE,
          // The first half of extra time.
          EXTRA_FIRST_HALF,
          // Half time between first and second extra halves.
          EXTRA_HALF_TIME,
          // The second half of extra time is about to start.
          // A kickoff is called within this stage.
          // This stage ends with the NORMAL_START.
          EXTRA_SECOND_HALF_PRE,
          // The second half of extra time.
          EXTRA_SECOND_HALF,
          // The break before penalty shootout.
          PENALTY_SHOOTOUT_BREAK,
          // The penalty shootout.
          PENALTY_SHOOTOUT,
          // The game is over.
          POST_GAME, REFBOX_STAGE_COUNT);

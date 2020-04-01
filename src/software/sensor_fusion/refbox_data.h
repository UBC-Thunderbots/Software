#pragma once

#include <vector>

#include "software/new_geom/point.h"
#include "software/proto/ssl_referee.pb.h"
#include "software/sensor_fusion/team_info.h"
#include "software/time/timestamp.h"
#include "software/util/printable_enum_macro/printable_enum_macro.h"

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

/**
 * RefboxData is a container class to represent the referee proto internally
 * It renames fields for ease of use, i.e. command->game_state and blue/yellow->us/them
 * and uses friendly data structures, such as Timestamp, Duration, and Point
 */
class RefboxData
{
   public:
    RefboxData(Timestamp packet_timestamp, Timestamp game_state_timestamp,
               int game_state_counter, Point designated_position,
               bool blue_team_on_positive_half,
               Duration current_game_state_time_remaining, TeamInfo friendly_team_info,
               TeamInfo enemy_team_info, RefboxGameState game_state,
               RefboxGameState next_game_state, RefboxStage stage,
               std::vector<GameEvent> game_events,
               std::vector<ProposedGameEvent> proposed_game_events);

    Timestamp getPacketTimestamp(void) const;

    Timestamp getGameStateTimestamp(void) const;

    int getGameStateCounter(void) const;

    Point getDesignatedPosition(void) const;

    bool getBlueTeamOnPositiveHalf(void) const;

    Duration getCurrentGameStateTimeRemaining(void) const;

    TeamInfo getFriendlyTeamInfo(void) const;

    TeamInfo getEnemyTeamInfo(void) const;

    RefboxGameState getGameState(void) const;

    RefboxGameState getNextGameState(void) const;

    RefboxStage getStage(void) const;

    bool operator==(const RefboxData &other) const;

   private:
    // packet timestamp
    Timestamp packet_timestamp;
    // time that current game state was initiated
    Timestamp game_state_timestamp;
    // number of game states issued since startup (mod 2^32)
    int game_state_counter;

    // ball placement position
    Point designated_position;
    // whether or not blue team is on positive half
    bool blue_team_on_positive_half;
    // time remaining for current game_state
    Duration current_game_state_time_remaining;

    // Referee's info on friendly team
    TeamInfo friendly_team_info;
    // Referee's info on enemy team
    TeamInfo enemy_team_info;
    // Current game state
    RefboxGameState game_state;
    // The game state that will be issued after the current stoppage and ball placement to
    // continue the game.
    RefboxGameState next_game_state;
    // Current stage of play
    RefboxStage stage;

    // Note: we're using (Proposed)GameEvent proto
    // directly instead of wrapper because YAGNI
    // We don't see a use for these events currently
    // but they must be included for completeness

    // All game events that were detected since the last RUNNING state.
    // Will be cleared as soon as the game is continued.
    std::vector<GameEvent> game_events;
    // All non-finished proposed game events that may be processed next.
    std::vector<ProposedGameEvent> proposed_game_events;
};

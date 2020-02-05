#pragma once

#include <ostream>
#include <vector>

#include "software/new_geom/point.h"
#include "software/proto/ssl_referee.pb.h"
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

struct TeamInfo
{
    // The team's name (empty string if operator has not typed anything).
    std::string name;
    // The number of goals scored by the team during normal play and overtime.
    int score;
    // The number of red cards issued to the team since the beginning of the game.
    int red_cards;
    // The amount of time (in microseconds) left on each yellow card issued to the team.
    // If no yellow cards are issued, this array has no elements.
    // Otherwise, times are ordered from smallest to largest.
    std::vector<int> yellow_card_times;
    // The total number of yellow cards ever issued to the team.
    int yellow_cards;
    // The number of timeouts this team can still call.
    // If in a timeout right now, that timeout is excluded.
    int timeouts;
    // The number of microseconds of timeout this team can use.
    int timeout_time;
    // The pattern number of this team's goalkeeper.
    int goalkeeper;
    // The total number of countable fouls that act towards yellow cards
    int foul_counter;
    // The number of consecutive ball placement failures of this team
    int ball_placement_failures;
    // Indicate if the team is able and allowed to place the ball
    bool can_place_ball;
    // The maximum number of bots allowed on the field based on division and cards
    int max_allowed_bots;
};

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
               std::vector<ProposedGameEvent> proposed_game_events)
        : packet_timestamp(packet_timestamp),
          game_state_timestamp(game_state_timestamp),
          game_state_counter(game_state_counter),
          designated_position(designated_position),
          blue_team_on_positive_half(blue_team_on_positive_half),
          current_game_state_time_remaining(current_game_state_time_remaining),
          friendly_team_info(friendly_team_info),
          enemy_team_info(enemy_team_info),
          game_state(game_state),
          next_game_state(next_game_state),
          stage(stage),
          game_events(game_events),
          proposed_game_events(proposed_game_events)
    {
    }

    Timestamp getPacketTimestamp(void) const
    {
        return packet_timestamp;
    }

    Timestamp getGameStateTimestamp(void) const
    {
        return game_state_timestamp;
    }

    int getGameStateCounter(void) const
    {
        return game_state_counter;
    }

    Point getDesignatedPosition(void) const
    {
        return designated_position;
    }

    bool getBlueTeamOnPositiveHalf(void) const
    {
        return blue_team_on_positive_half;
    }

    Duration getCurrentGameStateTimeRemaining(void) const
    {
        return current_game_state_time_remaining;
    }

    TeamInfo getFriendlyTeamInfo(void) const
    {
        return friendly_team_info;
    }

    TeamInfo getEnemyTeamInfo(void) const
    {
        return enemy_team_info;
    }

    RefboxGameState getGameState(void) const
    {
        return game_state;
    }

    RefboxGameState getNextGameState(void) const
    {
        return next_game_state;
    }

    RefboxStage getStage(void) const
    {
        return stage;
    }

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

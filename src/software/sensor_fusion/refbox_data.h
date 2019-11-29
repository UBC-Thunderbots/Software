#pragma once

#include <ostream>
#include <vector>

#include "software/new_geom/point.h"
#include "software/proto/ssl_referee.pb.h"
#include "software/util/time/timestamp.h"

enum class RefboxGameState
{
    // these enum items map to the constants in RefboxCommand.msg
    HALT                 = 0,
    STOP                 = 1,
    NORMAL_START         = 2,
    FORCE_START          = 3,
    PREPARE_KICKOFF_US   = 4,
    PREPARE_KICKOFF_THEM = 5,
    PREPARE_PENALTY_US   = 6,
    PREPARE_PENALTY_THEM = 7,
    DIRECT_FREE_US       = 8,
    DIRECT_FREE_THEM     = 9,
    INDIRECT_FREE_US     = 10,
    INDIRECT_FREE_THEM   = 11,
    TIMEOUT_US           = 12,
    TIMEOUT_THEM         = 13,
    GOAL_US              = 14,
    GOAL_THEM            = 15,
    BALL_PLACEMENT_US    = 16,
    BALL_PLACEMENT_THEM  = 17,
    REFBOX_GAME_STATE_COUNT
};

enum class RefboxStage
{
    // The first half is about to start.
    // A kickoff is called within this stage.
    // This stage ends with the NORMAL_START.
    NORMAL_FIRST_HALF_PRE = 0,
    // The first half of the normal game, before half time.
    NORMAL_FIRST_HALF = 1,
    // Half time between first and second halves.
    NORMAL_HALF_TIME = 2,
    // The second half is about to start.
    // A kickoff is called within this stage.
    // This stage ends with the NORMAL_START.
    NORMAL_SECOND_HALF_PRE = 3,
    // The second half of the normal game, after half time.
    NORMAL_SECOND_HALF = 4,
    // The break before extra time.
    EXTRA_TIME_BREAK = 5,
    // The first half of extra time is about to start.
    // A kickoff is called within this stage.
    // This stage ends with the NORMAL_START.
    EXTRA_FIRST_HALF_PRE = 6,
    // The first half of extra time.
    EXTRA_FIRST_HALF = 7,
    // Half time between first and second extra halves.
    EXTRA_HALF_TIME = 8,
    // The second half of extra time is about to start.
    // A kickoff is called within this stage.
    // This stage ends with the NORMAL_START.
    EXTRA_SECOND_HALF_PRE = 9,
    // The second half of extra time.
    EXTRA_SECOND_HALF = 10,
    // The break before penalty shootout.
    PENALTY_SHOOTOUT_BREAK = 11,
    // The penalty shootout.
    PENALTY_SHOOTOUT = 12,
    // The game is over.
    POST_GAME = 13,
    REFBOX_STAGE_COUNT
};

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

/**
 * Returns the name of the given refbox gamestate
 *
 * @param state The refbox gamestate to get the name of
 * @return The name of the given refbox gamestate
 */
std::string name(const RefboxGameState& state);
std::ostream& operator<<(std::ostream& os, const RefboxGameState& state);

/**
 * Returns the name of the given refbox gamestage
 *
 * @param stage The refbox gamestage to get the name of
 * @return The name of the given refbox gamestage
 */
std::string name(const RefboxStage& stage);
std::ostream& operator<<(std::ostream& os, const RefboxStage& stage);

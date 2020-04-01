#include "software/sensor_fusion/refbox_data.h"

RefboxData::RefboxData(Timestamp packet_timestamp, Timestamp game_state_timestamp,
                       int game_state_counter, Point designated_position,
                       bool blue_team_on_positive_half,
                       Duration current_game_state_time_remaining,
                       TeamInfo friendly_team_info, TeamInfo enemy_team_info,
                       RefboxGameState game_state, RefboxGameState next_game_state,
                       RefboxStage stage, std::vector<GameEvent> game_events,
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

Timestamp RefboxData::getPacketTimestamp(void) const
{
    return packet_timestamp;
}

Timestamp RefboxData::getGameStateTimestamp(void) const
{
    return game_state_timestamp;
}

int RefboxData::getGameStateCounter(void) const
{
    return game_state_counter;
}

Point RefboxData::getDesignatedPosition(void) const
{
    return designated_position;
}

bool RefboxData::getBlueTeamOnPositiveHalf(void) const
{
    return blue_team_on_positive_half;
}

Duration RefboxData::getCurrentGameStateTimeRemaining(void) const
{
    return current_game_state_time_remaining;
}

TeamInfo RefboxData::getFriendlyTeamInfo(void) const
{
    return friendly_team_info;
}

TeamInfo RefboxData::getEnemyTeamInfo(void) const
{
    return enemy_team_info;
}

RefboxGameState RefboxData::getGameState(void) const
{
    return game_state;
}

RefboxGameState RefboxData::getNextGameState(void) const
{
    return next_game_state;
}

RefboxStage RefboxData::getStage(void) const
{
    return stage;
}

bool RefboxData::operator==(const RefboxData &other) const
{
    return (
        (packet_timestamp == other.getPacketTimestamp()) &&
        (game_state_timestamp == other.getGameStateTimestamp()) &&
        (game_state_counter == other.getGameStateCounter()) &&
        (designated_position == other.getDesignatedPosition()) &&
        (blue_team_on_positive_half == other.getBlueTeamOnPositiveHalf()) &&
        (current_game_state_time_remaining == other.getCurrentGameStateTimeRemaining()) &&
        (friendly_team_info == other.getFriendlyTeamInfo()) &&
        (enemy_team_info == other.getEnemyTeamInfo()) &&
        (game_state == other.getGameState()) &&
        (next_game_state == other.getNextGameState()) && (stage == other.getStage()));
}

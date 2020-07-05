#include "software/proto/message_translation/ssl_referee.h"

// this maps a protobuf SSL_Referee_Command enum to its equivalent internal type
// this map is used when we are on the blue team
const static std::unordered_map<SSL_Referee::Command, RefboxGameState>
    blue_team_command_map = {
        {SSL_Referee_Command_HALT, RefboxGameState::HALT},
        {SSL_Referee_Command_STOP, RefboxGameState::STOP},
        {SSL_Referee_Command_NORMAL_START, RefboxGameState::NORMAL_START},
        {SSL_Referee_Command_FORCE_START, RefboxGameState::FORCE_START},
        {SSL_Referee_Command_PREPARE_KICKOFF_BLUE, RefboxGameState::PREPARE_KICKOFF_US},
        {SSL_Referee_Command_PREPARE_KICKOFF_YELLOW,
         RefboxGameState::PREPARE_KICKOFF_THEM},
        {SSL_Referee_Command_PREPARE_PENALTY_BLUE, RefboxGameState::PREPARE_PENALTY_US},
        {SSL_Referee_Command_PREPARE_PENALTY_YELLOW,
         RefboxGameState::PREPARE_PENALTY_THEM},
        {SSL_Referee_Command_DIRECT_FREE_BLUE, RefboxGameState::DIRECT_FREE_US},
        {SSL_Referee_Command_DIRECT_FREE_YELLOW, RefboxGameState::DIRECT_FREE_THEM},
        {SSL_Referee_Command_INDIRECT_FREE_BLUE, RefboxGameState::INDIRECT_FREE_US},
        {SSL_Referee_Command_INDIRECT_FREE_YELLOW, RefboxGameState::INDIRECT_FREE_THEM},
        {SSL_Referee_Command_TIMEOUT_BLUE, RefboxGameState::TIMEOUT_US},
        {SSL_Referee_Command_TIMEOUT_YELLOW, RefboxGameState::TIMEOUT_THEM},
        {SSL_Referee_Command_GOAL_BLUE, RefboxGameState::GOAL_US},
        {SSL_Referee_Command_GOAL_YELLOW, RefboxGameState::GOAL_THEM},
        {SSL_Referee_Command_BALL_PLACEMENT_BLUE, RefboxGameState::BALL_PLACEMENT_US},
        {SSL_Referee_Command_BALL_PLACEMENT_YELLOW,
         RefboxGameState::BALL_PLACEMENT_THEM}};

// this maps a protobuf SSL_Referee_Command enum to its equivalent internal type
// this map is used when we are on the yellow team
const static std::unordered_map<SSL_Referee::Command, RefboxGameState>
    yellow_team_command_map = {
        {SSL_Referee_Command_HALT, RefboxGameState::HALT},
        {SSL_Referee_Command_STOP, RefboxGameState::STOP},
        {SSL_Referee_Command_NORMAL_START, RefboxGameState::NORMAL_START},
        {SSL_Referee_Command_FORCE_START, RefboxGameState::FORCE_START},
        {SSL_Referee_Command_PREPARE_KICKOFF_BLUE, RefboxGameState::PREPARE_KICKOFF_THEM},
        {SSL_Referee_Command_PREPARE_KICKOFF_YELLOW, RefboxGameState::PREPARE_KICKOFF_US},
        {SSL_Referee_Command_PREPARE_PENALTY_BLUE, RefboxGameState::PREPARE_PENALTY_THEM},
        {SSL_Referee_Command_PREPARE_PENALTY_YELLOW, RefboxGameState::PREPARE_PENALTY_US},
        {SSL_Referee_Command_DIRECT_FREE_BLUE, RefboxGameState::DIRECT_FREE_THEM},
        {SSL_Referee_Command_DIRECT_FREE_YELLOW, RefboxGameState::DIRECT_FREE_US},
        {SSL_Referee_Command_INDIRECT_FREE_BLUE, RefboxGameState::INDIRECT_FREE_THEM},
        {SSL_Referee_Command_INDIRECT_FREE_YELLOW, RefboxGameState::INDIRECT_FREE_US},
        {SSL_Referee_Command_TIMEOUT_BLUE, RefboxGameState::TIMEOUT_THEM},
        {SSL_Referee_Command_TIMEOUT_YELLOW, RefboxGameState::TIMEOUT_US},
        {SSL_Referee_Command_GOAL_BLUE, RefboxGameState::GOAL_THEM},
        {SSL_Referee_Command_GOAL_YELLOW, RefboxGameState::GOAL_US},
        {SSL_Referee_Command_BALL_PLACEMENT_BLUE, RefboxGameState::BALL_PLACEMENT_THEM},
        {SSL_Referee_Command_BALL_PLACEMENT_YELLOW, RefboxGameState::BALL_PLACEMENT_US}};

RefboxGameState createRefboxGameState(const SSL_Referee &packet, TeamColour team_colour)
{
    if (team_colour == TeamColour::YELLOW)
    {
        return yellow_team_command_map.at(packet.command());
    }
    else
    {
        return blue_team_command_map.at(packet.command());
    }
}

// this maps a protobuf SSL_Referee_Stage enum to its RefboxStage equivalent
const static std::unordered_map<SSL_Referee::Stage, RefboxStage> refbox_stage_map = {
    {SSL_Referee_Stage_NORMAL_FIRST_HALF_PRE, RefboxStage::NORMAL_FIRST_HALF_PRE},
    {SSL_Referee_Stage_NORMAL_FIRST_HALF, RefboxStage::NORMAL_FIRST_HALF},
    {SSL_Referee_Stage_NORMAL_HALF_TIME, RefboxStage::NORMAL_HALF_TIME},
    {SSL_Referee_Stage_NORMAL_SECOND_HALF_PRE, RefboxStage::NORMAL_SECOND_HALF_PRE},
    {SSL_Referee_Stage_NORMAL_SECOND_HALF, RefboxStage::NORMAL_SECOND_HALF},
    {SSL_Referee_Stage_EXTRA_TIME_BREAK, RefboxStage::EXTRA_TIME_BREAK},
    {SSL_Referee_Stage_EXTRA_FIRST_HALF_PRE, RefboxStage::EXTRA_FIRST_HALF_PRE},
    {SSL_Referee_Stage_EXTRA_FIRST_HALF, RefboxStage::EXTRA_FIRST_HALF},
    {SSL_Referee_Stage_EXTRA_HALF_TIME, RefboxStage::EXTRA_HALF_TIME},
    {SSL_Referee_Stage_EXTRA_SECOND_HALF_PRE, RefboxStage::EXTRA_SECOND_HALF_PRE},
    {SSL_Referee_Stage_EXTRA_SECOND_HALF, RefboxStage::EXTRA_SECOND_HALF},
    {SSL_Referee_Stage_PENALTY_SHOOTOUT_BREAK, RefboxStage::PENALTY_SHOOTOUT_BREAK},
    {SSL_Referee_Stage_PENALTY_SHOOTOUT, RefboxStage::PENALTY_SHOOTOUT},
    {SSL_Referee_Stage_POST_GAME, RefboxStage::POST_GAME}};

RefboxStage createRefboxStage(const SSL_Referee &packet)
{
    return refbox_stage_map.at(packet.stage());
}

std::optional<Point> getBallPlacementPoint(const SSL_Referee &packet)
{
    if (packet.has_designated_position())
    {
        return Point(static_cast<double>(packet.designated_position().x()),
                     static_cast<double>(packet.designated_position().y()));
    }
    else
    {
        return std::nullopt;
    }
}

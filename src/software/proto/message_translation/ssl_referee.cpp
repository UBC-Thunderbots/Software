#include "software/proto/message_translation/ssl_referee.h"

// this maps a protobuf SSL::SSL_Referee_Command enum to its equivalent internal type
// this map is used when we are on the blue team
const static std::unordered_map<SSL::SSL_Referee::Command, RefereeCommand>
    blue_team_command_map = {
        {SSL::SSL_Referee_Command_HALT, RefereeCommand::HALT},
        {SSL::SSL_Referee_Command_STOP, RefereeCommand::STOP},
        {SSL::SSL_Referee_Command_NORMAL_START, RefereeCommand::NORMAL_START},
        {SSL::SSL_Referee_Command_FORCE_START, RefereeCommand::FORCE_START},
        {SSL::SSL_Referee_Command_PREPARE_KICKOFF_BLUE,
         RefereeCommand::PREPARE_KICKOFF_US},
        {SSL::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW,
         RefereeCommand::PREPARE_KICKOFF_THEM},
        {SSL::SSL_Referee_Command_PREPARE_PENALTY_BLUE,
         RefereeCommand::PREPARE_PENALTY_US},
        {SSL::SSL_Referee_Command_PREPARE_PENALTY_YELLOW,
         RefereeCommand::PREPARE_PENALTY_THEM},
        {SSL::SSL_Referee_Command_DIRECT_FREE_BLUE, RefereeCommand::DIRECT_FREE_US},
        {SSL::SSL_Referee_Command_DIRECT_FREE_YELLOW, RefereeCommand::DIRECT_FREE_THEM},
        {SSL::SSL_Referee_Command_INDIRECT_FREE_BLUE, RefereeCommand::INDIRECT_FREE_US},
        {SSL::SSL_Referee_Command_INDIRECT_FREE_YELLOW,
         RefereeCommand::INDIRECT_FREE_THEM},
        {SSL::SSL_Referee_Command_TIMEOUT_BLUE, RefereeCommand::TIMEOUT_US},
        {SSL::SSL_Referee_Command_TIMEOUT_YELLOW, RefereeCommand::TIMEOUT_THEM},
        {SSL::SSL_Referee_Command_GOAL_BLUE, RefereeCommand::GOAL_US},
        {SSL::SSL_Referee_Command_GOAL_YELLOW, RefereeCommand::GOAL_THEM},
        {SSL::SSL_Referee_Command_BALL_PLACEMENT_BLUE, RefereeCommand::BALL_PLACEMENT_US},
        {SSL::SSL_Referee_Command_BALL_PLACEMENT_YELLOW,
         RefereeCommand::BALL_PLACEMENT_THEM}};

// this maps a protobuf SSL::SSL_Referee_Command enum to its equivalent internal type
// this map is used when we are on the yellow team
const static std::unordered_map<SSL::SSL_Referee::Command, RefereeCommand>
    yellow_team_command_map = {
        {SSL::SSL_Referee_Command_HALT, RefereeCommand::HALT},
        {SSL::SSL_Referee_Command_STOP, RefereeCommand::STOP},
        {SSL::SSL_Referee_Command_NORMAL_START, RefereeCommand::NORMAL_START},
        {SSL::SSL_Referee_Command_FORCE_START, RefereeCommand::FORCE_START},
        {SSL::SSL_Referee_Command_PREPARE_KICKOFF_BLUE,
         RefereeCommand::PREPARE_KICKOFF_THEM},
        {SSL::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW,
         RefereeCommand::PREPARE_KICKOFF_US},
        {SSL::SSL_Referee_Command_PREPARE_PENALTY_BLUE,
         RefereeCommand::PREPARE_PENALTY_THEM},
        {SSL::SSL_Referee_Command_PREPARE_PENALTY_YELLOW,
         RefereeCommand::PREPARE_PENALTY_US},
        {SSL::SSL_Referee_Command_DIRECT_FREE_BLUE, RefereeCommand::DIRECT_FREE_THEM},
        {SSL::SSL_Referee_Command_DIRECT_FREE_YELLOW, RefereeCommand::DIRECT_FREE_US},
        {SSL::SSL_Referee_Command_INDIRECT_FREE_BLUE, RefereeCommand::INDIRECT_FREE_THEM},
        {SSL::SSL_Referee_Command_INDIRECT_FREE_YELLOW, RefereeCommand::INDIRECT_FREE_US},
        {SSL::SSL_Referee_Command_TIMEOUT_BLUE, RefereeCommand::TIMEOUT_THEM},
        {SSL::SSL_Referee_Command_TIMEOUT_YELLOW, RefereeCommand::TIMEOUT_US},
        {SSL::SSL_Referee_Command_GOAL_BLUE, RefereeCommand::GOAL_THEM},
        {SSL::SSL_Referee_Command_GOAL_YELLOW, RefereeCommand::GOAL_US},
        {SSL::SSL_Referee_Command_BALL_PLACEMENT_BLUE,
         RefereeCommand::BALL_PLACEMENT_THEM},
        {SSL::SSL_Referee_Command_BALL_PLACEMENT_YELLOW,
         RefereeCommand::BALL_PLACEMENT_US}};

RefereeCommand createRefereeCommand(const SSL::SSL_Referee &packet,
                                    TeamColour team_colour)
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

// this maps a protobuf SSL::SSL_Referee_Stage enum to its RefereeStage equivalent
const static std::unordered_map<SSL::SSL_Referee::Stage, RefereeStage> referee_stage_map =
    {{SSL::SSL_Referee_Stage_NORMAL_FIRST_HALF_PRE, RefereeStage::NORMAL_FIRST_HALF_PRE},
     {SSL::SSL_Referee_Stage_NORMAL_FIRST_HALF, RefereeStage::NORMAL_FIRST_HALF},
     {SSL::SSL_Referee_Stage_NORMAL_HALF_TIME, RefereeStage::NORMAL_HALF_TIME},
     {SSL::SSL_Referee_Stage_NORMAL_SECOND_HALF_PRE,
      RefereeStage::NORMAL_SECOND_HALF_PRE},
     {SSL::SSL_Referee_Stage_NORMAL_SECOND_HALF, RefereeStage::NORMAL_SECOND_HALF},
     {SSL::SSL_Referee_Stage_EXTRA_TIME_BREAK, RefereeStage::EXTRA_TIME_BREAK},
     {SSL::SSL_Referee_Stage_EXTRA_FIRST_HALF_PRE, RefereeStage::EXTRA_FIRST_HALF_PRE},
     {SSL::SSL_Referee_Stage_EXTRA_FIRST_HALF, RefereeStage::EXTRA_FIRST_HALF},
     {SSL::SSL_Referee_Stage_EXTRA_HALF_TIME, RefereeStage::EXTRA_HALF_TIME},
     {SSL::SSL_Referee_Stage_EXTRA_SECOND_HALF_PRE, RefereeStage::EXTRA_SECOND_HALF_PRE},
     {SSL::SSL_Referee_Stage_EXTRA_SECOND_HALF, RefereeStage::EXTRA_SECOND_HALF},
     {SSL::SSL_Referee_Stage_PENALTY_SHOOTOUT_BREAK,
      RefereeStage::PENALTY_SHOOTOUT_BREAK},
     {SSL::SSL_Referee_Stage_PENALTY_SHOOTOUT, RefereeStage::PENALTY_SHOOTOUT},
     {SSL::SSL_Referee_Stage_POST_GAME, RefereeStage::POST_GAME}};

RefereeStage createRefereeStage(const SSL::SSL_Referee &packet)
{
    return referee_stage_map.at(packet.stage());
}

std::optional<Point> getBallPlacementPoint(const SSL::SSL_Referee &packet)
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

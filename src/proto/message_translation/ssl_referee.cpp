#include "proto/message_translation/ssl_referee.h"

#include "shared/constants.h"

// this maps a protobuf SSLProto::Referee_Command enum to its equivalent internal type
// this map is used when we are on the blue team
const static std::unordered_map<SSLProto::Referee::Command, RefereeCommand>
    blue_team_command_map = {
        {SSLProto::Referee_Command_HALT, RefereeCommand::HALT},
        {SSLProto::Referee_Command_STOP, RefereeCommand::STOP},
        {SSLProto::Referee_Command_NORMAL_START, RefereeCommand::NORMAL_START},
        {SSLProto::Referee_Command_FORCE_START, RefereeCommand::FORCE_START},
        {SSLProto::Referee_Command_PREPARE_KICKOFF_BLUE,
         RefereeCommand::PREPARE_KICKOFF_US},
        {SSLProto::Referee_Command_PREPARE_KICKOFF_YELLOW,
         RefereeCommand::PREPARE_KICKOFF_THEM},
        {SSLProto::Referee_Command_PREPARE_PENALTY_BLUE,
         RefereeCommand::PREPARE_PENALTY_US},
        {SSLProto::Referee_Command_PREPARE_PENALTY_YELLOW,
         RefereeCommand::PREPARE_PENALTY_THEM},
        {SSLProto::Referee_Command_DIRECT_FREE_BLUE, RefereeCommand::DIRECT_FREE_US},
        {SSLProto::Referee_Command_DIRECT_FREE_YELLOW, RefereeCommand::DIRECT_FREE_THEM},
        {SSLProto::Referee_Command_INDIRECT_FREE_BLUE, RefereeCommand::INDIRECT_FREE_US},
        {SSLProto::Referee_Command_INDIRECT_FREE_YELLOW,
         RefereeCommand::INDIRECT_FREE_THEM},
        {SSLProto::Referee_Command_TIMEOUT_BLUE, RefereeCommand::TIMEOUT_US},
        {SSLProto::Referee_Command_TIMEOUT_YELLOW, RefereeCommand::TIMEOUT_THEM},
        {SSLProto::Referee_Command_BALL_PLACEMENT_BLUE,
         RefereeCommand::BALL_PLACEMENT_US},
        {SSLProto::Referee_Command_BALL_PLACEMENT_YELLOW,
         RefereeCommand::BALL_PLACEMENT_THEM}};

// this maps a protobuf SSLProto::Referee_Command enum to its equivalent internal type
// this map is used when we are on the yellow team
const static std::unordered_map<SSLProto::Referee::Command, RefereeCommand>
    yellow_team_command_map = {
        {SSLProto::Referee_Command_HALT, RefereeCommand::HALT},
        {SSLProto::Referee_Command_STOP, RefereeCommand::STOP},
        {SSLProto::Referee_Command_NORMAL_START, RefereeCommand::NORMAL_START},
        {SSLProto::Referee_Command_FORCE_START, RefereeCommand::FORCE_START},
        {SSLProto::Referee_Command_PREPARE_KICKOFF_BLUE,
         RefereeCommand::PREPARE_KICKOFF_THEM},
        {SSLProto::Referee_Command_PREPARE_KICKOFF_YELLOW,
         RefereeCommand::PREPARE_KICKOFF_US},
        {SSLProto::Referee_Command_PREPARE_PENALTY_BLUE,
         RefereeCommand::PREPARE_PENALTY_THEM},
        {SSLProto::Referee_Command_PREPARE_PENALTY_YELLOW,
         RefereeCommand::PREPARE_PENALTY_US},
        {SSLProto::Referee_Command_DIRECT_FREE_BLUE, RefereeCommand::DIRECT_FREE_THEM},
        {SSLProto::Referee_Command_DIRECT_FREE_YELLOW, RefereeCommand::DIRECT_FREE_US},
        {SSLProto::Referee_Command_INDIRECT_FREE_BLUE,
         RefereeCommand::INDIRECT_FREE_THEM},
        {SSLProto::Referee_Command_INDIRECT_FREE_YELLOW,
         RefereeCommand::INDIRECT_FREE_US},
        {SSLProto::Referee_Command_TIMEOUT_BLUE, RefereeCommand::TIMEOUT_THEM},
        {SSLProto::Referee_Command_TIMEOUT_YELLOW, RefereeCommand::TIMEOUT_US},
        {SSLProto::Referee_Command_BALL_PLACEMENT_BLUE,
         RefereeCommand::BALL_PLACEMENT_THEM},
        {SSLProto::Referee_Command_BALL_PLACEMENT_YELLOW,
         RefereeCommand::BALL_PLACEMENT_US}};

RefereeCommand createRefereeCommand(const SSLProto::Referee::Command &command,
                                    TeamColour team_colour)
{
    if (team_colour == TeamColour::YELLOW)
    {
        return yellow_team_command_map.at(command);
    }
    else
    {
        return blue_team_command_map.at(command);
    }
}

// this maps a protobuf SSLProto::Referee_Stage enum to its RefereeStage equivalent
const static std::unordered_map<SSLProto::Referee::Stage, RefereeStage>
    referee_stage_map = {
        {SSLProto::Referee_Stage_NORMAL_FIRST_HALF_PRE,
         RefereeStage::NORMAL_FIRST_HALF_PRE},
        {SSLProto::Referee_Stage_NORMAL_FIRST_HALF, RefereeStage::NORMAL_FIRST_HALF},
        {SSLProto::Referee_Stage_NORMAL_HALF_TIME, RefereeStage::NORMAL_HALF_TIME},
        {SSLProto::Referee_Stage_NORMAL_SECOND_HALF_PRE,
         RefereeStage::NORMAL_SECOND_HALF_PRE},
        {SSLProto::Referee_Stage_NORMAL_SECOND_HALF, RefereeStage::NORMAL_SECOND_HALF},
        {SSLProto::Referee_Stage_EXTRA_TIME_BREAK, RefereeStage::EXTRA_TIME_BREAK},
        {SSLProto::Referee_Stage_EXTRA_FIRST_HALF_PRE,
         RefereeStage::EXTRA_FIRST_HALF_PRE},
        {SSLProto::Referee_Stage_EXTRA_FIRST_HALF, RefereeStage::EXTRA_FIRST_HALF},
        {SSLProto::Referee_Stage_EXTRA_HALF_TIME, RefereeStage::EXTRA_HALF_TIME},
        {SSLProto::Referee_Stage_EXTRA_SECOND_HALF_PRE,
         RefereeStage::EXTRA_SECOND_HALF_PRE},
        {SSLProto::Referee_Stage_EXTRA_SECOND_HALF, RefereeStage::EXTRA_SECOND_HALF},
        {SSLProto::Referee_Stage_PENALTY_SHOOTOUT_BREAK,
         RefereeStage::PENALTY_SHOOTOUT_BREAK},
        {SSLProto::Referee_Stage_PENALTY_SHOOTOUT, RefereeStage::PENALTY_SHOOTOUT},
        {SSLProto::Referee_Stage_POST_GAME, RefereeStage::POST_GAME}};

RefereeStage createRefereeStage(const SSLProto::Referee &packet)
{
    return referee_stage_map.at(packet.stage());
}

std::optional<Point> getBallPlacementPoint(const SSLProto::Referee &packet)
{
    if (packet.has_designated_position())
    {
        return Point(
            static_cast<double>(packet.designated_position().x() * METERS_PER_MILLIMETER),
            static_cast<double>(packet.designated_position().y() *
                                METERS_PER_MILLIMETER));
    }
    else
    {
        return std::nullopt;
    }
}

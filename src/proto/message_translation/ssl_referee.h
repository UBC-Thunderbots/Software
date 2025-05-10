#pragma once

#include "proto/ssl_gc_referee_message.pb.h"
#include "software/geom/point.h"
#include "software/world/game_state.h"
#include "software/world/team_types.h"


namespace ssl_referee {

/**
 * @brief this maps a protobuf SSLProto::Referee_Command enum to its equivalent internal type
 * @note this map is used when we are on the blue team
 */
inline const static std::unordered_map<SSLProto::Referee::Command, RefereeCommand>
        blue_team_command_map = {
        {SSLProto::Referee_Command_HALT,                   RefereeCommand::HALT},
        {SSLProto::Referee_Command_STOP,                   RefereeCommand::STOP},
        {SSLProto::Referee_Command_NORMAL_START,           RefereeCommand::NORMAL_START},
        {SSLProto::Referee_Command_FORCE_START,            RefereeCommand::FORCE_START},
        {SSLProto::Referee_Command_PREPARE_KICKOFF_BLUE,   RefereeCommand::PREPARE_KICKOFF_US},
        {SSLProto::Referee_Command_PREPARE_KICKOFF_YELLOW, RefereeCommand::PREPARE_KICKOFF_THEM},
        {SSLProto::Referee_Command_PREPARE_PENALTY_BLUE,   RefereeCommand::PREPARE_PENALTY_US},
        {SSLProto::Referee_Command_PREPARE_PENALTY_YELLOW, RefereeCommand::PREPARE_PENALTY_THEM},
        {SSLProto::Referee_Command_DIRECT_FREE_BLUE,       RefereeCommand::DIRECT_FREE_US},
        {SSLProto::Referee_Command_DIRECT_FREE_YELLOW,     RefereeCommand::DIRECT_FREE_THEM},
        {SSLProto::Referee_Command_INDIRECT_FREE_BLUE,     RefereeCommand::INDIRECT_FREE_US},
        {SSLProto::Referee_Command_INDIRECT_FREE_YELLOW,   RefereeCommand::INDIRECT_FREE_THEM},
        {SSLProto::Referee_Command_TIMEOUT_BLUE,           RefereeCommand::TIMEOUT_US},
        {SSLProto::Referee_Command_TIMEOUT_YELLOW,         RefereeCommand::TIMEOUT_THEM},
        {SSLProto::Referee_Command_BALL_PLACEMENT_BLUE,    RefereeCommand::BALL_PLACEMENT_US},
        {SSLProto::Referee_Command_BALL_PLACEMENT_YELLOW,  RefereeCommand::BALL_PLACEMENT_THEM}};

/**
 * @brief this set contains an ongoing list of deprecated SSLProto::Referee_Commands
 */
inline const static std::unordered_set<SSLProto::Referee::Command> deprecated_commands = {
        SSLProto::Referee_Command_GOAL_YELLOW,
        SSLProto::Referee_Command_GOAL_BLUE,
        SSLProto::Referee_Command_INDIRECT_FREE_YELLOW,
        SSLProto::Referee_Command_INDIRECT_FREE_BLUE,
};

/**
 * @brief this maps a protobuf SSLProto::Referee_Command enum to its equivalent internal type
 * @note this map is used when we are on the yellow team
 */
inline const static std::unordered_map<SSLProto::Referee::Command, RefereeCommand>
        yellow_team_command_map = {
        {SSLProto::Referee_Command_HALT,                   RefereeCommand::HALT},
        {SSLProto::Referee_Command_STOP,                   RefereeCommand::STOP},
        {SSLProto::Referee_Command_NORMAL_START,           RefereeCommand::NORMAL_START},
        {SSLProto::Referee_Command_FORCE_START,            RefereeCommand::FORCE_START},
        {SSLProto::Referee_Command_PREPARE_KICKOFF_BLUE,   RefereeCommand::PREPARE_KICKOFF_THEM},
        {SSLProto::Referee_Command_PREPARE_KICKOFF_YELLOW, RefereeCommand::PREPARE_KICKOFF_US},
        {SSLProto::Referee_Command_PREPARE_PENALTY_BLUE,   RefereeCommand::PREPARE_PENALTY_THEM},
        {SSLProto::Referee_Command_PREPARE_PENALTY_YELLOW, RefereeCommand::PREPARE_PENALTY_US},
        {SSLProto::Referee_Command_DIRECT_FREE_BLUE,       RefereeCommand::DIRECT_FREE_THEM},
        {SSLProto::Referee_Command_DIRECT_FREE_YELLOW,     RefereeCommand::DIRECT_FREE_US},
        {SSLProto::Referee_Command_INDIRECT_FREE_BLUE,     RefereeCommand::INDIRECT_FREE_THEM},
        {SSLProto::Referee_Command_INDIRECT_FREE_YELLOW,   RefereeCommand::INDIRECT_FREE_US},
        {SSLProto::Referee_Command_TIMEOUT_BLUE,           RefereeCommand::TIMEOUT_THEM},
        {SSLProto::Referee_Command_TIMEOUT_YELLOW,         RefereeCommand::TIMEOUT_US},
        {SSLProto::Referee_Command_BALL_PLACEMENT_BLUE,    RefereeCommand::BALL_PLACEMENT_THEM},
        {SSLProto::Referee_Command_BALL_PLACEMENT_YELLOW,  RefereeCommand::BALL_PLACEMENT_US}};


/**
 * @brief this maps a protobuf SSLProto::Referee_Stage enum to its RefereeStage equivalent
 */
inline const static std::unordered_map<SSLProto::Referee::Stage, RefereeStage>
        referee_stage_map = {
        {SSLProto::Referee_Stage_NORMAL_FIRST_HALF_PRE,
                                                     RefereeStage::NORMAL_FIRST_HALF_PRE},
        {SSLProto::Referee_Stage_NORMAL_FIRST_HALF,  RefereeStage::NORMAL_FIRST_HALF},
        {SSLProto::Referee_Stage_NORMAL_HALF_TIME,   RefereeStage::NORMAL_HALF_TIME},
        {SSLProto::Referee_Stage_NORMAL_SECOND_HALF_PRE,
                                                     RefereeStage::NORMAL_SECOND_HALF_PRE},
        {SSLProto::Referee_Stage_NORMAL_SECOND_HALF, RefereeStage::NORMAL_SECOND_HALF},
        {SSLProto::Referee_Stage_EXTRA_TIME_BREAK,   RefereeStage::EXTRA_TIME_BREAK},
        {SSLProto::Referee_Stage_EXTRA_FIRST_HALF_PRE,
                                                     RefereeStage::EXTRA_FIRST_HALF_PRE},
        {SSLProto::Referee_Stage_EXTRA_FIRST_HALF,   RefereeStage::EXTRA_FIRST_HALF},
        {SSLProto::Referee_Stage_EXTRA_HALF_TIME,    RefereeStage::EXTRA_HALF_TIME},
        {SSLProto::Referee_Stage_EXTRA_SECOND_HALF_PRE,
                                                     RefereeStage::EXTRA_SECOND_HALF_PRE},
        {SSLProto::Referee_Stage_EXTRA_SECOND_HALF,  RefereeStage::EXTRA_SECOND_HALF},
        {SSLProto::Referee_Stage_PENALTY_SHOOTOUT_BREAK,
                                                     RefereeStage::PENALTY_SHOOTOUT_BREAK},
        {SSLProto::Referee_Stage_PENALTY_SHOOTOUT,   RefereeStage::PENALTY_SHOOTOUT},
        {SSLProto::Referee_Stage_POST_GAME,          RefereeStage::POST_GAME}};

/**
 * Converts a SSLProto::Referee packet into a RefereeCommand for the GameController
 * command contained in the packet, based on which team we are (blue or yellow).
 *
 * @param packet SSLProto::Referee protobuf
 * @param team_colour the team colour to get game state for
 *
 * @return RefereeCommand from command
 */
RefereeCommand createRefereeCommand(const SSLProto::Referee &packet,
                                    TeamColour team_colour);

/**
 * Converts a SSLProto::Referee protobuf Stage contained in the SSLProto::Referee packet
 * into a RefereeStage
 *
 * @param packet SSLProto::Referee protobuf
 *
 * @return RefereeStage from stage
 */
RefereeStage createRefereeStage(const SSLProto::Referee &packet);

/**
 * Extracts the ball placement point from SSLProto::Referee packet
 *
 * @param packet SSLProto::Referee protobuf
 *
 * @return ball placement point if found
 */
std::optional<Point> getBallPlacementPoint(const SSLProto::Referee &packet);
}
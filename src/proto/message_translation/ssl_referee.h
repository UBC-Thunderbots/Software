#pragma once

#include "proto/ssl_gc_referee_message.pb.h"
#include "software/geom/point.h"
#include "software/world/game_state.h"
#include "software/world/team_types.h"

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

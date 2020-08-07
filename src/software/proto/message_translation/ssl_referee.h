#pragma once

#include "software/geom/point.h"
#include "software/proto/ssl_referee.pb.h"
#include "software/world/game_state.h"
#include "software/world/team_colour.h"

/**
 * Converts a SSL_Referee packet into a RefereeCommand for the Refbox command contained
 * in the packet, based on which team we are (blue or yellow).
 *
 * @param packet SSL_Referee protobuf
 * @param team_colour the team colour to get game state for
 *
 * @return RefereeCommand from command
 */
RefereeCommand createRefereeCommand(const SSL_Referee &packet, TeamColour team_colour);

/**
 * Converts a SSL_Referee protobuf Stage contained in the SSL_Referee packet into a
 * RefereeStage
 *
 * @param packet SSL_Referee protobuf
 *
 * @return RefereeStage from stage
 */
RefereeStage createRefereeStage(const SSL_Referee &packet);

/**
 * Extracts the ball placement point from SSL_Referee packet
 *
 * @param packet SSL_Referee protobuf
 *
 * @return ball placement point if found
 */
std::optional<Point> getBallPlacementPoint(const SSL_Referee &packet);

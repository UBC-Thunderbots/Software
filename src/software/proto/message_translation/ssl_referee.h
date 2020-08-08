#pragma once

#include "software/geom/point.h"
#include "software/proto/ssl_gc_referee_message.pb.h"
#include "software/world/game_state.h"
#include "software/world/team_colour.h"

/**
 * Converts a SSL::Referee packet into a RefereeCommand for the GameController command
 * contained in the packet, based on which team we are (blue or yellow).
 *
 * @param packet SSL::Referee protobuf
 * @param team_colour the team colour to get game state for
 *
 * @return RefereeCommand from command
 */
RefereeCommand createRefereeCommand(const SSL::Referee &packet, TeamColour team_colour);

/**
 * Converts a SSL::Referee protobuf Stage contained in the SSL::Referee packet
 * into a RefereeStage
 *
 * @param packet SSL::Referee protobuf
 *
 * @return RefereeStage from stage
 */
RefereeStage createRefereeStage(const SSL::Referee &packet);

/**
 * Extracts the ball placement point from SSL::Referee packet
 *
 * @param packet SSL::Referee protobuf
 *
 * @return ball placement point if found
 */
std::optional<Point> getBallPlacementPoint(const SSL::Referee &packet);

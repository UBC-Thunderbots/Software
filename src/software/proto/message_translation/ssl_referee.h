#pragma once

#include "software/new_geom/point.h"
#include "software/proto/ssl_referee.pb.h"
#include "software/sensor_fusion/refbox_data.h"
#include "software/world/team_colour.h"

/**
 * Converts a SSL_Referee packet into a RefboxGameState for the Refbox command contained
 * in the packet, based on which team we are (blue or yellow).
 *
 * @param packet SSL_Referee protobuf
 * @param team_colour the team colour to get game state for
 *
 * @return RefboxGameState from command
 */
RefboxGameState createRefboxGameState(const SSL_Referee &packet, TeamColour team_colour);

/**
 * Converts a SSL_Referee protobuf Stage contained in the SSL_Referee packet into a
 * RefboxStage
 *
 * @param packet SSL_Referee protobuf
 *
 * @return RefboxStage from stage
 */
RefboxStage createRefboxStage(const SSL_Referee &packet);

/**
 * Extracts the ball placement point from SSL_Referee packet
 *
 * @param packet SSL_Referee protobuf
 *
 * @return ball placement point if found
 */
std::optional<Point> getBallPlacementPoint(const SSL_Referee &packet);

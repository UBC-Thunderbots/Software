#pragma once

#include "software/proto/ssl_referee.pb.h"
#include "software/sensor_fusion/refbox_data.h"
#include "software/world/team_colour.h"

/**
 * Converts a Referee packet into a RefboxGameState for the Refbox command contained
 * in the packet, based on which team we are (blue or yellow).
 *
 * @param packet Referee protobuf
 * @param team_colour the team colour to get game state for
 *
 * @return RefboxGameState from command
 */
RefboxGameState createRefboxGameState(const Referee &packet, TeamColour team_colour);

/**
 * Converts a Referee protobuf Stage contained in the Referee packet into a
 * RefboxStage
 *
 * @param packet Referee protobuf
 *
 * @return RefboxStage from stage
 */
RefboxStage createRefboxStage(const Referee &packet);

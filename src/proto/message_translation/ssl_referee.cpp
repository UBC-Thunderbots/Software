#include "proto/message_translation/ssl_referee.h"

#include "shared/constants.h"

namespace ssl_referee {

RefereeCommand createRefereeCommand(const SSLProto::Referee &packet,
                                                 TeamColour team_colour) {
    if (team_colour == TeamColour::YELLOW) {
        return yellow_team_command_map.at(packet.command());
    } else {
        return blue_team_command_map.at(packet.command());
    }
}

RefereeStage createRefereeStage(const SSLProto::Referee &packet) {
    return referee_stage_map.at(packet.stage());
}

std::optional<Point> getBallPlacementPoint(const SSLProto::Referee &packet) {
    if (packet.has_designated_position()) {
        return Point(
                static_cast<double>(packet.designated_position().x() * METERS_PER_MILLIMETER),
                static_cast<double>(packet.designated_position().y() *
                                    METERS_PER_MILLIMETER));
    } else {
        return std::nullopt;
    }
}
}

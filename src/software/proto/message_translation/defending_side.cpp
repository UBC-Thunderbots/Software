#include "software/proto/message_translation/team_side.h"

std::unique_ptr<TeamSideMsg> createTeamSideMsg(bool defending_positive_side) {
    auto team_side_msg                    = std::make_unique<TeamSideMsg>();
    team_side_msg->set_defending_positive_side(defending_positive_side);

    return team_side_msg;
}

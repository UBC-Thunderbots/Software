#include "software/ai/hl/stp/play/crease_defense/crease_defense_play_fsm.h"

CreaseDefensePlayFSM ::CreaseDefensePlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config), crease_defenders({})
{
}

void CreaseDefensePlayFSM::defendDefenseArea(const Update& event)
{
    unsigned int num_defenders = event.common.num_tactics;
    if (num_defenders > 3)
    {
        LOG(WARNING) << "CreaseDefensePlay can only handle up to 3 crease defenders"
                     << std::endl;
        num_defenders = 3;
    }

    if (num_defenders != crease_defenders.size())
    {
        setUpDefenders(num_defenders);
    }
    static const std::vector<TbotsProto::CreaseDefenderAlignment> ALIGNMENTS = {
        TbotsProto::CreaseDefenderAlignment::CENTRE,
        TbotsProto::CreaseDefenderAlignment::LEFT,
        TbotsProto::CreaseDefenderAlignment::RIGHT};
    for (unsigned int i = 0; i < crease_defenders.size(); i++)
    {
        crease_defenders.at(i)->updateControlParams(
            event.control_params.enemy_threat_origin, ALIGNMENTS.at(i),
            event.control_params.max_allowed_speed_mode);
    }
    PriorityTacticVector tactics_to_return = {{}};
    tactics_to_return[0].insert(tactics_to_return[0].end(), crease_defenders.begin(),
                                crease_defenders.end());
    event.common.set_tactics(tactics_to_return);
}

void CreaseDefensePlayFSM::setUpDefenders(unsigned int num_defenders)
{
    crease_defenders = std::vector<std::shared_ptr<CreaseDefenderTactic>>(num_defenders);
    std::generate(crease_defenders.begin(), crease_defenders.end(), [this]() {
        return std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config());
    });
}

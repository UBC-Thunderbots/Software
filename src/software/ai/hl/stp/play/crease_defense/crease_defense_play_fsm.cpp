#include "software/ai/hl/stp/play/crease_defense/crease_defense_play_fsm.h"

void CreaseDefensePlayFSM::defendDefenseArea(const Update& event)
{
    if (event.control_params.num_additional_crease_defenders_tactics + 1 !=
        crease_defenders.size())
    {
        setUpDefenders(event.control_params.num_additional_crease_defenders_tactics + 1);
    }
    // TODO (#2385): Make the crease defense align robots on either side of the threat
    // when there are an even number of robots
    static const std::vector<CreaseDefenderAlignment> ALIGNMENTS = {
        CreaseDefenderAlignment::CENTRE, CreaseDefenderAlignment::LEFT,
        CreaseDefenderAlignment::RIGHT};
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
            play_config->getRobotNavigationObstacleConfig());
    });
}

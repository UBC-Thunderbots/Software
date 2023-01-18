#include "software/ai/hl/stp/play/defense/defense_play_fsm.h"

#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/defense_position.h"

DefensePlayFSM ::DefensePlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config), crease_defenders({}), pass_defenders({})
{
}

void DefensePlayFSM::defendAgainstThreats(const Update& event) 
{
    auto enemy_threats = getAllEnemyThreats(
        event.common.world.field(), event.common.world.friendlyTeam(),
        event.common.world.enemyTeam(), event.common.world.ball(), false);

    // Remove all threats which are not "immediate" (i.e not near the friendly
    // half of the field). Threats can be 1/4 of the field length outside of our 
    // friendly half and still be considered immediate
    auto field = event.common.world.field();
    enemy_threats.erase(std::remove_if(enemy_threats.begin(), enemy_threats.end(), 
                        [&field](const auto enemy_threat) {
                            return enemy_threat.robot.position().x() >= 
                                   field.centerPoint().x() + (field.xLength() / 4);
                        }), enemy_threats.end());

    if (enemy_threats.size() == 0) 
    {
        return;
    }

    auto positions = getAllDefensePositions(enemy_threats, event.common.world.field());

    // Choose which defense positions to assign defenders to based on number 
    // of tactics available to set
    std::vector<DefensePosition> crease_defense_positions;
    std::vector<DefensePosition> pass_defense_positions;
    for (unsigned int i = 0; i < event.common.num_tactics; i++) 
    {
        DefensePosition defense_position;
        if (i < positions.size())
        {
            defense_position = positions.at(i);
        }
        else
        {
            // If we have more tactics to set than determined defense positions, 
            // assign remaining defenders to the defense position with the
            // highest "effectiveness"
            defense_position = positions.front();
        }

        if (defense_position.is_crease_defense) 
        {
            crease_defense_positions.emplace_back(defense_position);

            // If we have at least two available defenders, two defenders should
            // be assigned to the highest scoring crease defense position to better
            // block the shot cone of the most threatening enemy
            if (i == 0 && event.common.num_tactics >= 2)
            {
                crease_defense_positions.emplace_back(defense_position);
                i++;
            }
        }
        else 
        {
            pass_defense_positions.emplace_back(defense_position);
        }
    }

    // Reset tactics if the number of crease defenders or pass defenders 
    // we intend to assign has changed
    auto num_crease_defenders = static_cast<unsigned int>(crease_defense_positions.size());
    auto num_pass_defenders = static_cast<unsigned int>(pass_defense_positions.size());
    if (num_crease_defenders != crease_defenders.size())
    {
        setUpCreaseDefenders(num_crease_defenders);
    }
    if (num_pass_defenders != pass_defenders.size())
    {
        setUpPassDefenders(num_pass_defenders);
    }

    for (unsigned int i = 0; i < crease_defenders.size(); i++)
    {
        crease_defenders.at(i)->updateControlParams(
            crease_defense_positions.at(i).position,
            TbotsProto::CreaseDefenderAlignment::CENTRE,
            event.control_params.max_allowed_speed_mode);
    }
    
    for (unsigned int i = 0; i < pass_defenders.size(); i++)
    {
        pass_defenders.at(i)->updateControlParams(
            pass_defense_positions.at(i).position);
    }
    
    PriorityTacticVector tactics_to_return = {{},{}};
    tactics_to_return[0].insert(tactics_to_return[0].end(), crease_defenders.begin(),
                                crease_defenders.end());
    tactics_to_return[1].insert(tactics_to_return[1].end(), pass_defenders.begin(),
                                pass_defenders.end());
    event.common.set_tactics(tactics_to_return);
}

void DefensePlayFSM::setUpCreaseDefenders(unsigned int num_crease_defenders)
{
    crease_defenders = std::vector<std::shared_ptr<CreaseDefenderTactic>>(num_crease_defenders);
    std::generate(crease_defenders.begin(), crease_defenders.end(), [this]() {
        return std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config());
    });
}

void DefensePlayFSM::setUpPassDefenders(unsigned int num_pass_defenders)
{
    pass_defenders = std::vector<std::shared_ptr<PassDefenderTactic>>(num_pass_defenders);
    std::generate(pass_defenders.begin(), pass_defenders.end(), [this]() {
        return std::make_shared<PassDefenderTactic>();
    });
}

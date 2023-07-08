#include "software/ai/hl/stp/play/enemy_free_kick_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/world/game_state.h"

EnemyFreekickPlay::EnemyFreekickPlay(TbotsProto::AiConfig config) : Play(config, true),
  defense_play(std::make_shared<DefensePlay>(ai_config)),
  crease_defense(std::make_shared<CreaseDefensePlay>(ai_config))
{
}

void EnemyFreekickPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const World &world)
{
    // Block free kicker
    auto block_free_kicker = std::make_shared<PassDefenderTactic>();

    do
    {
        int num_friendly_robots = static_cast<int>(world.friendlyTeam().numRobots());
        bool assign_pass_defender = true;
        PriorityTacticVector tactics_to_return = {{}};

        std::vector<EnemyThreat> enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), false);
        std::queue defense_assignments = getAllDefenderAssignments(enemy_threats, world.field(), world.ball(),
                ai_config.defense_play_config().defender_assignment_config());

        // figure out how many robots get assigned to what state
        int num_defenders = static_cast<int>(defense_assignments.size()) + 2;
        if (num_defenders > num_friendly_robots)
        {
            num_defenders = num_friendly_robots;
        }
        // prefer a crease defender if we only have one defending robot
        if (num_defenders == 1 || enemy_threats.size() == 0)
        {
            assign_pass_defender = false;
        }
        int num_shooters = num_friendly_robots - num_defenders;
        if (assign_pass_defender)
        {
            if (num_shooters > 0)
            {
                num_shooters--;
            }
            else if (num_defenders > 0)
            {
                num_defenders--;
            }
        }

        defense_play->updateControlParams(defense_assignments, TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

        defense_play->updateTactics(PlayUpdate(
                    world, num_defenders,
                    [&tactics_to_return](PriorityTacticVector new_tactics)
                    {
                        for (const auto& tactic_vector : new_tactics)
                        {
                            tactics_to_return.push_back(tactic_vector);
                        }
                    },
                    {},
                    [](InterPlayCommunication comm){}));


        if (assign_pass_defender)
        {
            // block free kick
            Vector block_direction =
                Vector::createFromAngle(enemy_threats[0].robot.orientation());
            Point block_kick_point =
                world.ball().position() +
                block_direction.normalize(0.5 + 2 * ROBOT_MAX_RADIUS_METERS);
            block_free_kicker->updateControlParams(block_kick_point);
            if (tactics_to_return.size() > 0 && tactics_to_return[0].size() > 0 && assign_pass_defender)
            {
                tactics_to_return[0].insert(tactics_to_return[0].begin()+1, block_free_kicker); 
            }
            else if (assign_pass_defender && tactics_to_return.size() > 0)
            {
                tactics_to_return[0].push_back(block_free_kicker);
            }
            else
            {
                tactics_to_return.push_back({block_free_kicker});
            }
        }

        crease_defense->updateTactics(PlayUpdate(
                    world, num_friendly_robots-num_defenders, 
                    [&tactics_to_return](PriorityTacticVector new_tactics)
                    {
                        for (const auto& tactic_vector : new_tactics)
                        {
                            tactics_to_return.push_back(tactic_vector);
                        }
                    },
                    InterPlayCommunication(),
                    [](InterPlayCommunication comm){}));

        yield(tactics_to_return);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, EnemyFreekickPlay, TbotsProto::AiConfig>
    factory;

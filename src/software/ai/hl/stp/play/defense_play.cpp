#include "software/ai/hl/stp/play/defense_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/world/game_state.h"
#include "software/world/team.h"

DefensePlay::DefensePlay(TbotsProto::AiConfig config) : Play(config, true) {}

void DefensePlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    auto shoot_goal_tactic = std::make_shared<AttackerTactic>(ai_config);

    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        // TODO-AKHIL remove hardcoded values
        std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config()),
        std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config())};

    std::array<std::shared_ptr<ShadowEnemyTactic>, 2> shadow_enemy_tactics = {
        std::make_shared<ShadowEnemyTactic>(),
        std::make_shared<ShadowEnemyTactic>(),
    };

    auto move_tactics = std::vector<std::shared_ptr<MoveTactic>>{
        std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>()};

    std::vector<std::shared_ptr<StopTactic>> stop_tactics = {
        std::make_shared<StopTactic>(false), std::make_shared<StopTactic>(false)};

    do
    {
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);

        PriorityTacticVector result = {{shoot_goal_tactic}};

        // Update crease defenders
        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::LEFT);
        result[0].emplace_back(std::get<0>(crease_defender_tactics));
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::RIGHT);
        result[0].emplace_back(std::get<1>(crease_defender_tactics));

        // Determine how many "immediate" enemy threats there are. If there is only one we
        // have both shadow enemy tactics swarm and block the "immediate" threat.
        // Otherwise we assign ShadowEnemy tactics for the next highest threats. If there
        // any extra friendly robots, have them perform a reasonable default defensive
        // tactic
        int immediate_enemy_threats = static_cast<int>(std::count_if(
            enemy_threats.begin(), enemy_threats.end(), [this, world](auto enemy_threat) {
                return distance(world.field().friendlyGoal(),
                                enemy_threat.robot.position()) <
                       ai_config.defense_play_config().immediate_threat_distance();
            }));


        if (immediate_enemy_threats == 1)
        {
            std::get<0>(shadow_enemy_tactics)
                ->updateControlParams(enemy_threats.at(0),
                                      ROBOT_SHADOWING_DISTANCE_METERS);
            std::get<1>(shadow_enemy_tactics)
                ->updateControlParams(enemy_threats.at(0),
                                      ROBOT_SHADOWING_DISTANCE_METERS);
            result[0].insert(result[0].end(), shadow_enemy_tactics.begin(),
                             shadow_enemy_tactics.end());
        }
        else
        {
            if (enemy_threats.size() > 0)
            {
                std::get<0>(shadow_enemy_tactics)
                    ->updateControlParams(enemy_threats.at(0),
                                          ROBOT_SHADOWING_DISTANCE_METERS);
                result[0].emplace_back(std::get<0>(shadow_enemy_tactics));
            }
            else
            {
                result[0].emplace_back(move_tactics[0]);
            }

            if (enemy_threats.size() > 1)
            {
                std::get<1>(shadow_enemy_tactics)
                    ->updateControlParams(enemy_threats.at(1),
                                          ROBOT_SHADOWING_DISTANCE_METERS);
                result[0].emplace_back(std::get<1>(shadow_enemy_tactics));
            }
            else
            {
                auto nearest_enemy_robot =
                    world.enemyTeam().getNearestRobot(world.ball().position());
                if (nearest_enemy_robot)
                {
                    // Blocks in front of where the closest enemy robot is
                    Point block_point =
                        nearest_enemy_robot->position() +
                        Vector::createFromAngle(nearest_enemy_robot->orientation()) *
                            ROBOT_SHADOWING_DISTANCE_METERS;
                    move_tactics[1]->updateControlParams(
                        block_point, nearest_enemy_robot->orientation() + Angle::half(),
                        0.0);
                    result[0].emplace_back(move_tactics[1]);
                }
                else
                {
                    LOG(WARNING)
                        << "There are no enemy robots so a MoveTactic is not being assigned";
                }
            }
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, DefensePlay, TbotsProto::AiConfig> factory;

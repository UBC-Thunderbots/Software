#include "software/ai/hl/stp/play/enemy_free_kick_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/world/game_state.h"

EnemyFreekickPlay::EnemyFreekickPlay(TbotsProto::AiConfig config) : Play(config, true) {}

void EnemyFreekickPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const World &world)
{
    // Free kicks are usually taken near the edge of the field. We assign one robot to
    // shadow the enemy robot taking the free kick
    auto shadow_free_kicker = std::make_shared<ShadowEnemyTactic>();

    // Assign up to two crease defenders
    std::vector<std::shared_ptr<CreaseDefenderTactic>> crease_defenders = {
            std::make_shared<CreaseDefenderTactic>(
                    ai_config.robot_navigation_obstacle_config()),
            std::make_shared<CreaseDefenderTactic>(
                    ai_config.robot_navigation_obstacle_config())};

    // Try to shadow as many enemy robots as possible
    std::vector<std::shared_ptr<ShadowEnemyTactic>> shadow_potential_receivers;

    do
    {
        // Create tactic vector
        PriorityTacticVector tactics_to_run = {{}};

        // Get all enemy threats
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);

        if (enemy_threats.empty())
        {
            return;
        }

        // Assign crease defenders
        auto num_crease_defenders = world.friendlyTeam().numRobots() > 3 ? 2 : 1;
        if (num_crease_defenders == 2)
        {
            crease_defenders[0]->updateControlParams(
                    world.ball().position(), TbotsProto::CreaseDefenderAlignment::LEFT);
            crease_defenders[1]->updateControlParams(
                    world.ball().position(), TbotsProto::CreaseDefenderAlignment::RIGHT);

            tactics_to_run[0].emplace_back(crease_defenders[0]);
            tactics_to_run[0].emplace_back(crease_defenders[1]);
        }
        else
        {
            crease_defenders[0]->updateControlParams(
                    world.ball().position(), TbotsProto::CreaseDefenderAlignment::CENTRE);

            tactics_to_run[0].emplace_back(crease_defenders[0]);
        }

        auto num_unassigned_robots = 5 - num_crease_defenders;
        for (int i = 0;
             i < std::min(num_unassigned_robots, (int)enemy_threats.size() - 1); i++)
        {
            shadow_potential_receivers.emplace_back(
                    std::make_shared<ShadowEnemyTactic>());
            shadow_potential_receivers[i]->updateControlParams(
                    enemy_threats[i + 1], ROBOT_MAX_RADIUS_METERS * 3);

            tactics_to_run[0].emplace_back(shadow_potential_receivers[i]);
        }

        // Shadow free kicker should shadow the enemy robot with the ball
        shadow_free_kicker->updateControlParams(enemy_threats[0],
                                                ROBOT_MAX_RADIUS_METERS * 3);
        tactics_to_run[0].emplace_back(shadow_free_kicker);

        // yield the Tactics this Play wants to run, in order of priority
        yield(tactics_to_run);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, EnemyFreekickPlay, TbotsProto::AiConfig>
    factory;

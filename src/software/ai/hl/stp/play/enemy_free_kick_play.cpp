#include "software/ai/hl/stp/play/enemy_free_kick_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/world/game_state.h"
#include "software/geom/vector.h"

EnemyFreekickPlay::EnemyFreekickPlay(TbotsProto::AiConfig config) : Play(config, true) {}

void EnemyFreekickPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const World &world)
{
    // Free kicks are usually taken near the edge of the field. 
    // 1. We assign one robot to block the enemy robot taking the free kick
    // 2. We assign up to two crease defenders
    // 3. We try to block as many enemy robots as possible from receiving a pass
    
    auto block_free_kicker = std::make_shared<PassDefenderTactic>();

    // Assign up to two crease defenders
    std::vector<std::shared_ptr<CreaseDefenderTactic>> crease_defenders = {
        std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config()),
        std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config())};
    
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

        // Block free kicker
        Vector block_direction = Vector::createFromAngle(enemy_threats[0].robot.orientation());
        Point block_kick_point = world.ball().position() + block_direction.normalize(0.5 + 2 * ROBOT_MAX_RADIUS_METERS);
        block_free_kicker->updateControlParams(block_kick_point);
        tactics_to_run[0].emplace_back(block_free_kicker);

        // Assign crease defenders
        auto num_crease_defenders = world.friendlyTeam().numRobots() > 3 ? 2 : 1;
        if (num_crease_defenders == 2)
        {
            auto crease_defender_left = std::make_shared<CreaseDefenderTactic>(ai_config.robot_navigation_obstacle_config());
            auto crease_defender_right = std::make_shared<CreaseDefenderTactic>(ai_config.robot_navigation_obstacle_config());
            crease_defender_left->updateControlParams(
                world.ball().position(), TbotsProto::CreaseDefenderAlignment::LEFT);
            crease_defender_right->updateControlParams(
                world.ball().position(), TbotsProto::CreaseDefenderAlignment::RIGHT);

            tactics_to_run[0].emplace_back(crease_defender_left);
            tactics_to_run[0].emplace_back(crease_defender_right);
        }
        else
        {
            auto crease_defender_centre = std::make_shared<CreaseDefenderTactic>(ai_config.robot_navigation_obstacle_config());
            crease_defender_centre->updateControlParams(
                world.ball().position(), TbotsProto::CreaseDefenderAlignment::CENTRE);

            tactics_to_run[0].emplace_back(crease_defender_centre);
        }

        // Block potential receivers
        auto num_unassigned_robots = world.friendlyTeam().numRobots() - 2 - num_crease_defenders;
        int target_enemy_counter = 1;
        while (num_unassigned_robots > 0)
        {
            if (target_enemy_counter >= (int)enemy_threats.size()) {
                break;
            }
            // Do not try to block enemies that are too far back
            if (enemy_threats[target_enemy_counter].robot.position().x() >= world.ball().position().x() + 1) {
//                LOG(INFO) << "Defender too far. Not shadowing";
                target_enemy_counter++;
                continue;
            }
            auto block_potential_receiver = std::make_shared<PassDefenderTactic>();
            Point enemy_position = enemy_threats[target_enemy_counter].robot.position();

            Vector enemy_to_ball_vector = world.ball().position() - enemy_position;
            Point block_pass_point = enemy_position + enemy_to_ball_vector.normalize(ROBOT_MAX_RADIUS_METERS * 3);

            block_potential_receiver->updateControlParams(block_pass_point);

            tactics_to_run[0].emplace_back(block_potential_receiver);
            num_unassigned_robots--;
            target_enemy_counter++;
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(tactics_to_run);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, EnemyFreekickPlay, TbotsProto::AiConfig>
    factory;

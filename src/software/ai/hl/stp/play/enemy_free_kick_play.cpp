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
                                       const WorldPtr &world_ptr)
{
    // Init a Crease Defender Tactic
    auto crease_defender_tactic = std::make_shared<CreaseDefenderTactic>(
        ai_config.robot_navigation_obstacle_config());

    // These robots will both block the enemy robot taking a free kick
    std::array<std::shared_ptr<ShadowEnemyTactic>, 2> shadow_free_kicker = {
        std::make_shared<ShadowEnemyTactic>(), std::make_shared<ShadowEnemyTactic>()};

    // Init Shadow Enemy Tactics for extra robots
    std::array<std::shared_ptr<ShadowEnemyTactic>, 2> shadow_potential_receivers = {
        std::make_shared<ShadowEnemyTactic>(), std::make_shared<ShadowEnemyTactic>()};

    // Init Move Tactics for extra robots (These will be used if there are no robots to
    // shadow)
    auto move_tactic_main      = std::make_shared<MoveTactic>();
    auto move_tactic_secondary = std::make_shared<MoveTactic>();

    do
    {
        // Create tactic vector (starting with Goalie)
        PriorityTacticVector tactics_to_run = {{}};

        // Get all enemy threats
        auto enemy_threats = getAllEnemyThreats(world_ptr->field(), world_ptr->friendlyTeam(),
                                                world_ptr->enemyTeam(), world_ptr->ball(), false);

        // shadow free kicker should shadow the robot with the ball and if no such enemy
        // exists, then it will default to positioning between the ball and the friendly
        // defense area
        if (enemy_threats.size() >= 1)
        {
            std::get<0>(shadow_free_kicker)
                ->updateControlParams(enemy_threats.at(0), ROBOT_MAX_RADIUS_METERS * 3);
            std::get<1>(shadow_free_kicker)
                ->updateControlParams(enemy_threats.at(0), ROBOT_MAX_RADIUS_METERS * 3);
        }

        // Add Freekick shadower tactics
        tactics_to_run[0].emplace_back(std::get<0>(shadow_free_kicker));
        tactics_to_run[0].emplace_back(std::get<1>(shadow_free_kicker));
        // Add Crease defender tactic on side of open enemy threats
        if (enemy_threats.size() >= 4)
        {
            if (enemy_threats.at(3).robot.position().y() > 0)
            {
                crease_defender_tactic->updateControlParams(
                        world_ptr->ball().position(), TbotsProto::CreaseDefenderAlignment::LEFT);
            }
            else
            {
                crease_defender_tactic->updateControlParams(
                        world_ptr->ball().position(), TbotsProto::CreaseDefenderAlignment::RIGHT);
            }
        }
        else
        {
            crease_defender_tactic->updateControlParams(
                    world_ptr->ball().position(), TbotsProto::CreaseDefenderAlignment::CENTRE);
        }

        tactics_to_run[0].emplace_back(crease_defender_tactic);

        // Assign ShadowEnemy tactics until we have every enemy covered. If there are not
        // enough threats to shadow, move our robots to block the friendly net
        if (enemy_threats.size() <= 1)
        {
            // Since the first enemy threat is covered by the shadow_free_kicker, just
            // move to block the net
            move_tactic_main->updateControlParams(
                    world_ptr->field().friendlyGoalCenter() +
                    Vector(0, 2 * ROBOT_MAX_RADIUS_METERS),
                (world_ptr->ball().position() - world_ptr->field().friendlyGoalCenter())
                    .orientation(),
                0);
            move_tactic_main->updateControlParams(
                    world_ptr->field().friendlyGoalCenter() +
                    Vector(0, -2 * ROBOT_MAX_RADIUS_METERS),
                (world_ptr->ball().position() - world_ptr->field().friendlyGoalCenter())
                    .orientation(),
                0);

            tactics_to_run[0].emplace_back(move_tactic_main);
            tactics_to_run[0].emplace_back(move_tactic_secondary);
        }
        if (enemy_threats.size() == 2)
        {
            // Shadow the second most threatening enemy threat and move one robot to block
            // the net
            move_tactic_main->updateControlParams(
                    world_ptr->field().friendlyGoalCenter() +
                    Vector(0, 2 * ROBOT_MAX_RADIUS_METERS),
                (world_ptr->ball().position() - world_ptr->field().friendlyGoalCenter())
                    .orientation(),
                0);
            std::get<0>(shadow_potential_receivers)
                ->updateControlParams(enemy_threats.at(1), ROBOT_MAX_RADIUS_METERS * 3);

            tactics_to_run[0].emplace_back(std::get<0>(shadow_potential_receivers));
            tactics_to_run[0].emplace_back(move_tactic_main);
        }
        if (enemy_threats.size() >= 3)
        {
            // Shadow the second and third most threatening enemy threats
            std::get<0>(shadow_potential_receivers)
                ->updateControlParams(enemy_threats.at(1), ROBOT_MAX_RADIUS_METERS * 3);
            std::get<1>(shadow_potential_receivers)
                ->updateControlParams(enemy_threats.at(2), ROBOT_MAX_RADIUS_METERS * 3);

            tactics_to_run[0].emplace_back(std::get<0>(shadow_potential_receivers));
            tactics_to_run[0].emplace_back(std::get<1>(shadow_potential_receivers));
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(tactics_to_run);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, EnemyFreekickPlay, TbotsProto::AiConfig>
    factory;

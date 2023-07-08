#include "software/ai/hl/stp/play/enemy_free_kick_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/world/game_state.h"

EnemyFreekickPlay::EnemyFreekickPlay(TbotsProto::AiConfig config) : Play(config, true),
  defense_play(std::make_shared<DefensePlay>()),
  shoot_or_pass_play(std::make_shared<ShootOrPassPlay>())
{
}

void EnemyFreekickPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const World &world)
{
    do
    {
        PriorityTacticVector tactics_to_run = {{}};

        std::queue defense_assignments = getAllDefenderAssignments(getAllEnemyThreats(world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), false), world.field(), world.ball(),
                config.defense_play_config);

        int num_defenders = defense_play_assignments.size() + 2;
        if (num_defenders > world.friendlyTeam().numRobots())
        {
            num_defenders = world.friendlyTeam().numRobots();
        }

        defense_play->updateControlParams(defense_play_assignments, TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

        defense_play->updateTactics(PlayUpdate(
                    world, numRobots,
                    [&tactics_to_return](PriorityTacticVector new_tactics)
                    {
                        for (const auto& tactic_vector : new_tactics)
                        {
                            tactics_to_return.push_back(tactic_vector);
                        }
                    },
                    {},
                    [](InterPlayCommunication comm){}));


        shoot_or_pass_play->updateControlParams(

        yield(tactics_to_run);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, EnemyFreekickPlay, TbotsProto::AiConfig>
    factory;

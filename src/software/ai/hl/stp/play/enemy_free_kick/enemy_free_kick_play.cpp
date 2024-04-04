#include "software/ai/hl/stp/play/enemy_free_kick/enemy_free_kick_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/world/game_state.h"

EnemyFreeKickPlay::EnemyFreeKickPlay(TbotsProto::AiConfig config)
    : Play(config, true), fsm{EnemyFreeKickPlayFSM{config}}, control_params{}
{
}

void EnemyFreeKickPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const WorldPtr &world_ptr)
{
    // This function doesn't get called, it should be removed once coroutines
    // are phased out
    while (true)
    {
        yield({{}});
    }
}

void EnemyFreeKickPlay::updateControlParams(
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode)
{
    control_params.max_allowed_speed_mode = max_allowed_speed_mode;
}

void EnemyFreeKickPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(EnemyFreeKickPlayFSM::Update(control_params, play_update));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, EnemyFreeKickPlay, TbotsProto::AiConfig>
    factory;

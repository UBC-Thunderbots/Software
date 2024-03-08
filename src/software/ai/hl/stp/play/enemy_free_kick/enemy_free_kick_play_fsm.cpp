#include "software/ai/hl/stp/play/enemy_free_kick/enemy_free_kick_play_fsm.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/world/game_state.h"

EnemyFreeKickPlayFSM::EnemyFreeKickPlayFSM(TbotsProto::AiConfig config)
: ai_config(ai_config)
//  defense_play(std::make_shared<DefensePlay>(ai_config))
{
}


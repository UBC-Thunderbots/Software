#include "software/ai/hl/stp/play/kickoff_enemy/kickoff_enemy_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/util/generic_factory/generic_factory.h"

KickoffEnemyPlay::KickoffEnemyPlay(
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayBase<KickoffEnemyPlayFSM>(ai_config_ptr, false)
{
}

void KickoffEnemyPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                      const WorldPtr &world_ptr)
{
    // Does not get called.
    while (true)
    {
        yield({{}});
    }
}

void KickoffEnemyPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(KickoffEnemyPlayFSM::Update(control_params, play_update));
}

std::vector<std::string> KickoffEnemyPlay::getState()
{
    std::vector<std::string> state;
    state.emplace_back(objectTypeName(*this) + " - " + getCurrentFullStateName(fsm));
    return state;
}


// Register this play in the genericFactory
static TGenericFactory<std::string, Play, KickoffEnemyPlay,
                       std::shared_ptr<const TbotsProto::AiConfig>>
    factory;

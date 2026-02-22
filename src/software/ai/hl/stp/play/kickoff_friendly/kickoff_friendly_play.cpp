#include "software/ai/hl/stp/play/kickoff_friendly/kickoff_friendly_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/util/generic_factory/generic_factory.h"


KickoffFriendlyPlay::KickoffFriendlyPlay(
        std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
        :PlayBase<KickoffFriendlyPlayFSM>(ai_config_ptr, true)
{
}

void KickoffFriendlyPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                         const WorldPtr &world_ptr)
{
    // Does not get called.
    while (true)
    {
        yield({{}});
    }
}

void KickoffFriendlyPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(KickoffFriendlyPlayFSM::Update(control_params, play_update));
}

std::vector<std::string> KickoffFriendlyPlay::getState()
{
    std::vector<std::string> state;
    state.emplace_back(objectTypeName(*this) + " - " + getCurrentFullStateName(fsm));
    return state;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, KickoffFriendlyPlay,
        std::shared_ptr<const TbotsProto::AiConfig>> factory;

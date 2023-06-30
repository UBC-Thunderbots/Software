#include "software/ai/hl/stp/play/free_kick/free_kick_play.h"

#include "shared/constants.h"
#include "software/util/generic_factory/generic_factory.h"

FreeKickPlay::FreeKickPlay(TbotsProto::AiConfig config)
    : Play(config, true), fsm{FreeKickPlayFSM{config}}, control_params{}
{
}

void FreeKickPlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    // This function doesn't get called so it does nothing, will be removed once
    // coroutines are phased out

    /**
     * This play is basically:
     * - One robot attempts to shoot first. If there is no good shot, it will attempt to
     *   pass, and finally chips towards the enemy goal if it can't find a pass in time
     * - Two robots try to get in good positions near the enemy net to receive a pass
     * - Two robots crease defend
     * - One robot is goalie
     */
}

void FreeKickPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(FreeKickPlayFSM::Update(control_params, play_update));
}

std::vector<std::string> FreeKickPlay::getState()
{
    std::vector<std::string> state;
    state.emplace_back(objectTypeName(*this) + " - " + getCurrentFullStateName(fsm));
    return state;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, FreeKickPlay, TbotsProto::AiConfig> factory;
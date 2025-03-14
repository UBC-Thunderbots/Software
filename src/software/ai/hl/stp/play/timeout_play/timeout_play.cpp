#include "software/ai/hl/stp/play/timeout_play/timeout_play.h"

#include "software/util/generic_factory/generic_factory.h"

TimeoutPlay::TimeoutPlay(TbotsProto::AiConfig config)
    : Play(config, false /*requires_goalie*/),
      fsm(TimeoutPlayFSM(config)),
      control_params()
{
}

void TimeoutPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                 const WorldPtr &world_ptr)
{
    // This function doesn't get called and will be removed when coroutines are phased
    // out.
}

void TimeoutPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(TimeoutPlayFSM::Update(control_params, play_update));
}

static TGenericFactory<std::string, Play, TimeoutPlay, TbotsProto::AiConfig> factory;

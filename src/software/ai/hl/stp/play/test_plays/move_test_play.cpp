#include "software/ai/hl/stp/play/test_plays/move_test_play.h"

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

MoveTestPlay::MoveTestPlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayBase<MoveTestPlayFSM>(ai_config_ptr, false)
{
}

void MoveTestPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                  const WorldPtr &world_ptr)
{
    // This function doesn't get called, it should be removed once coroutines
    // are phased out
}

void MoveTestPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(MoveTestPlayFSM::Update(control_params, play_update));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, MoveTestPlay,
                       std::shared_ptr<const TbotsProto::AiConfig>>
    factory;

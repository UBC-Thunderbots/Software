#include "software/ai/hl/stp/play/stop_play.h"

#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

StopPlay::StopPlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayBase<StopPlayFSM>(ai_config_ptr, true)
{
    goalie_tactic = std::make_shared<GoalieTactic>(ai_config_ptr);
    goalie_tactic->updateMaxSpeedMode(TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND);
}

void StopPlay::getNextTactics(TacticCoroutine::push_type &yield,
                              const WorldPtr &world_ptr)
{
}

void StopPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(StopPlayFSM::Update(control_params, play_update));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, StopPlay,
                       std::shared_ptr<const TbotsProto::AiConfig>>
    factory;

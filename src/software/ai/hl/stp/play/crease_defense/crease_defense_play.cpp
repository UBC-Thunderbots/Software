#include "software/ai/hl/stp/play/crease_defense/crease_defense_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

CreaseDefensePlay::CreaseDefensePlay(std::shared_ptr<Strategy> strategy)
    : Play(true, strategy),
      fsm(std::make_unique<FSM<CreaseDefensePlayFSM>>(CreaseDefensePlayFSM(ai_config))),
      control_params{
          .enemy_threat_origin    = Point(),
          .max_allowed_speed_mode = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT}
{
}

void CreaseDefensePlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const World &world)
{
    // This function doesn't get called so it does nothing
    while (true)
    {
        yield({{}});
    }
}

void CreaseDefensePlay::reset()
{
    Play::reset();

    fsm = std::make_unique<FSM<CreaseDefensePlayFSM>>(CreaseDefensePlayFSM(ai_config));
}

void CreaseDefensePlay::updateControlParams(
    const Point &enemy_threat_origin,
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode)
{
    control_params.enemy_threat_origin    = enemy_threat_origin;
    control_params.max_allowed_speed_mode = max_allowed_speed_mode;
}

void CreaseDefensePlay::updateTactics(const PlayUpdate &play_update)
{
    fsm->process_event(CreaseDefensePlayFSM::Update(control_params, play_update));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, CreaseDefensePlay, std::shared_ptr<Strategy>> factory;

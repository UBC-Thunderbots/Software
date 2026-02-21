#include "software/ai/hl/stp/play/shoot_or_chip/shoot_or_chip_play.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/world/game_state.h"

ShootOrChipPlay::ShootOrChipPlay(
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayBase<ShootOrChipPlayFSM>(ai_config_ptr, true)
{
}

void ShootOrChipPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                     const WorldPtr &world_ptr)
{
}


void ShootOrChipPlay::updateTactics(const PlayUpdate &play_update)
{
    fsm.process_event(ShootOrChipPlayFSM::Update(control_params, play_update));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ShootOrChipPlay,
                       std::shared_ptr<const TbotsProto::AiConfig>>
    factory;

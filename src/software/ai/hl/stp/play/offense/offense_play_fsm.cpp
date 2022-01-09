#include "software/ai/hl/stp/play/offense/offense_play_fsm.h"

OffensePlayFSM::OffensePlayFSM(std::shared_ptr<const PlayConfig> play_config)
    : play_config(play_config),
      shoot_or_pass_play_fsm(
          std::make_shared<FSM<ShootOrPassPlayFSM>>(ShootOrPassPlayFSM{play_config})),
      crease_defense_play_fsm(
          std::make_shared<FSM<CreaseDefensePlayFSM>>(CreaseDefensePlayFSM{play_config}))
{
}

void OffensePlayFSM::updateOffense(const Update& event)
{
    PriorityTacticVector tactics_to_return;

    shoot_or_pass_play_fsm->process_event(ShootOrPassPlayFSM::Update(
        ShootOrPassPlayFSM::ControlParams{},
        PlayUpdate(event.common.world, 3,
                   [&tactics_to_return](PriorityTacticVector new_tactics) {
                       for (const auto& tactic_vector : new_tactics)
                       {
                           tactics_to_return.push_back(tactic_vector);
                       }
                   })));

    crease_defense_play_fsm->process_event(CreaseDefensePlayFSM::Update(
        CreaseDefensePlayFSM::ControlParams{
            .enemy_threat_origin    = event.common.world.ball().position(),
            .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT},
        PlayUpdate(event.common.world, 2,
                   [&tactics_to_return](PriorityTacticVector new_tactics) {
                       for (const auto& tactic_vector : new_tactics)
                       {
                           tactics_to_return.push_back(tactic_vector);
                       }
                   })));

    event.common.set_tactics(tactics_to_return);
}

bool OffensePlayFSM::doneOffense(const Update& event)
{
    return shoot_or_pass_play_fsm->is(boost::sml::X);
}

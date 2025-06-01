#include "software/ai/hl/stp/tactic/attacker/attacker_fsm.h"

#include "software/logger/logger.h"

void AttackerFSM::pivotKick(const Update& event,
                            boost::sml::back::process<PivotKickFSM::Update> processEvent)
{
    auto ball_position = event.common.world_ptr->ball().position();
    Point chip_target  = event.common.world_ptr->field().enemyGoalCenter();
    if (event.control_params.chip_target)
    {
        chip_target = event.control_params.chip_target.value();
    }
    // default to chipping the ball away
    PivotKickFSM::ControlParams control_params{
        .kick_origin       = ball_position,
        .kick_direction    = (chip_target - ball_position).orientation(),
        .auto_chip_or_kick = AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP,
                                            (chip_target - ball_position).length()}};

    if (event.control_params.shot)
    {
        // shoot on net
        control_params = PivotKickFSM::ControlParams{
            .kick_origin = ball_position,
            .kick_direction =
                (event.control_params.shot->getPointToShootAt() - ball_position)
                    .orientation(),
            .auto_chip_or_kick = AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                                BALL_MAX_SPEED_METERS_PER_SECOND}};
    }
    else if (event.control_params.pass_committed)
    {
        // we have committed to passing, execute the pass
        control_params = PivotKickFSM::ControlParams{
            .kick_origin    = event.control_params.best_pass_so_far->passerPoint(),
            .kick_direction = event.control_params.best_pass_so_far->passerOrientation(),
            .auto_chip_or_kick =
                AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                               event.control_params.best_pass_so_far->speed()}};
    }
    processEvent(PivotKickFSM::Update(control_params, event.common));
}


void AttackerFSM::keepAway(const Update& event,
                           boost::sml::back::process<KeepAwayFSM::Update> processEvent)
{
    KeepAwayFSM::ControlParams control_params{.best_pass_so_far =
                                                  event.control_params.best_pass_so_far};

    //LOG(INFO)<< event.common.robot.breakbeamTripped() <<event.common.robot.position()<< event.common.robot.id();
    processEvent(KeepAwayFSM::Update(control_params, event.common));
}


bool AttackerFSM::shouldKick(const Update& event)
{
    return event.control_params.pass_committed || event.control_params.shot;
}

#include "software/ai/hl/stp/tactic/one_touch_attacker/one_touch_attacker_fsm.h"

void OneTouchAttackerFSM::kickBall(const Update& event)
{
    auto ball_position = event.common.world.ball().position();
    Point chip_target  = event.common.world.field().enemyGoalCenter();
    if (event.control_params.chip_target)
    {
        chip_target = event.control_params.chip_target.value();
    }
    // default to chipping the ball away
    KickFSM::ControlParams control_params{
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
                                                BALL_MAX_SPEED_METERS_PER_SECOND - 0.5}};
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

bool OneTouchAttackerFSM::shouldKick(const Update& event)
{
    // check for enemy threat
    Circle about_to_steal_danger_zone(
        event.common.robot.position(),
        attacker_tactic_config.enemy_about_to_steal_ball_radius());
    for (const auto& enemy : event.common.world.enemyTeam().getAllRobots())
    {
        if (contains(about_to_steal_danger_zone, enemy.position()))
        {
            return true;
        }
    }
    // otherwise check for shot or pass committed
    return event.control_params.pass_committed || event.control_params.shot;
}

    void OneTouchAttackerFSM::kickBall(const Update& event){}
    void OneTouchAttackerFSM::alignToBall(const Update& event){}

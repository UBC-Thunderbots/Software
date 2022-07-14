#include "software/ai/hl/stp/tactic/move/move_fsm.h"

void MoveFSM::updateMove(const Update &event)
{
    event.common.set_primitive(createMovePrimitive(
            CREATE_MOTION_CONTROL(event.control_params.destination),
            event.control_params.final_orientation, event.control_params.final_speed,
            event.control_params.dribbler_mode, event.control_params.ball_collision_type,
            event.control_params.auto_chip_or_kick,
            event.control_params.max_allowed_speed_mode,
            event.control_params.target_spin_rev_per_s, event.common.robot.robotConstants(), std::optional<double>(),
            true));
}

bool MoveFSM::moveDone(const Update &event)
{
    return robotReachedDestination(event.common.robot, event.control_params.destination,
                                   event.control_params.final_orientation);
}

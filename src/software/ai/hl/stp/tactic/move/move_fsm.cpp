#include "software/ai/hl/stp/tactic/move/move_fsm.h"

#include "software/ai/hl/stp/primitive/move_primitive.h"

void MoveFSM::updateMove(const Update &event)
{
    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, event.control_params.destination,
        event.control_params.final_orientation,
        event.control_params.max_allowed_speed_mode, event.control_params.dribbler_mode,
        event.control_params.ball_collision_type,
        event.control_params.auto_chip_or_kick));
}

bool MoveFSM::moveDone(const Update &event)
{
    return robotReachedDestination(event.common.robot, event.control_params.destination,
                                   event.control_params.final_orientation);
}

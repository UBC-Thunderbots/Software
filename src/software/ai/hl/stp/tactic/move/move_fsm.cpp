#include "software/ai/hl/stp/tactic/move/move_fsm.h"

#include "software/ai/hl/stp/primitive/move_primitive.h"

void MoveFSM::updateMove(const Update &event)
{
    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, event.control_params.destination,
        event.control_params.final_orientation,
        event.control_params.max_allowed_speed_mode,
        event.control_params.obstacle_avoidance_mode, event.control_params.dribbler_mode,
        event.control_params.ball_collision_type,
        event.control_params.auto_chip_or_kick));
}

bool MoveFSM::moveDone(const Update &event)
{
    // only finish moving if not dribbling. Sometimes when dribbling we just want to hold
    // the ball somewhere.
    return event.control_params.dribbler_mode == TbotsProto::DribblerMode::OFF &&
           robotReachedDestination(event.common.robot, event.control_params.destination,
                                   event.control_params.final_orientation);
}

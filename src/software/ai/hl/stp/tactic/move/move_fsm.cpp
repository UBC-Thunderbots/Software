#include "software/ai/hl/stp/tactic/move/move_fsm.h"

#include "software/ai/hl/stp/tactic/move_primitive.h"

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
    return robotReachedDestination(event.common.robot, event.control_params.destination,
                                   event.control_params.final_orientation);
}


void MoveFSM::updateControlParams(
        Point destination, Angle final_orientation, TbotsProto::DribblerMode dribbler_mode,
        TbotsProto::BallCollisionType ball_collision_type, AutoChipOrKick auto_chip_or_kick,
        TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode,
        TbotsProto::ObstacleAvoidanceMode obstacle_avoidance_mode)
{
    // Update the control parameters stored by this Tactic
    control_params.destination             = destination;
    control_params.final_orientation       = final_orientation;
    control_params.dribbler_mode           = dribbler_mode;
    control_params.ball_collision_type     = ball_collision_type;
    control_params.auto_chip_or_kick       = auto_chip_or_kick;
    control_params.max_allowed_speed_mode  = max_allowed_speed_mode;
    control_params.obstacle_avoidance_mode = obstacle_avoidance_mode;
}

void MoveFSM::updateControlParams(
        Point destination, Angle final_orientation,
        TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode,
        TbotsProto::ObstacleAvoidanceMode obstacle_avoidance_mode)
{
    // Update the control parameters stored by this Tactic
    control_params.destination             = destination;
    control_params.final_orientation       = final_orientation;
    control_params.dribbler_mode           = TbotsProto::DribblerMode::OFF;
    control_params.ball_collision_type     = TbotsProto::BallCollisionType::AVOID;
    control_params.auto_chip_or_kick       = {AutoChipOrKickMode::OFF, 0};
    control_params.max_allowed_speed_mode  = max_allowed_speed_mode;
    control_params.obstacle_avoidance_mode = obstacle_avoidance_mode;
}


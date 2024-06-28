#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"

#include "software/ai/hl/stp/tactic/move_primitive.h"

void KickFSM::updateKick(const Update &event)
{
    Point ball_position = event.common.world_ptr->ball().position();
    Vector direction_to_kick =
        Vector::createFromAngle(event.control_params.kick_direction);
    Point kick_target =
        ball_position - direction_to_kick.normalize(DIST_TO_FRONT_OF_ROBOT_METERS - 0.01);

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, kick_target, event.control_params.kick_direction,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                       event.control_params.kick_speed_meters_per_second}));
}

void KickFSM::updateGetBehindBall(
    const Update &event, boost::sml::back::process<GetBehindBallFSM::Update> processEvent)
{
    GetBehindBallFSM::ControlParams control_params{
        .ball_location   = event.control_params.kick_origin,
        .chick_direction = event.control_params.kick_direction};

    // Update the get behind ball fsm
    processEvent(GetBehindBallFSM::Update(control_params, event.common));
}

bool KickFSM::ballChicked(const Update &event)
{
    return event.common.world_ptr->ball().hasBallBeenKicked(
        event.control_params.kick_direction);
}

bool KickFSM::robotAlignedForKick(const Update &event)
{
    return isRobotReadyToChick(event.common.robot,
                               event.common.world_ptr->ball().position(),
                               event.control_params.kick_direction);
}

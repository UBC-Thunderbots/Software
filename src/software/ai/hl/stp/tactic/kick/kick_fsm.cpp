#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"

#include "proto/parameters.pb.h"


void KickFSM::updateKick(const Update &event)
{
    event.common.set_primitive(createMovePrimitive(
        CREATE_MOTION_CONTROL(
            event.control_params.kick_origin +
            Vector::createFromAngle(event.control_params.kick_direction).normalize(0.5)),
        event.control_params.kick_direction, 0.1, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW, event.control_params.auto_chip_or_kick,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
        event.common.robot.robotConstants(), std::optional<double>(), true));
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
    return event.common.world.ball().hasBallBeenKicked(
        event.control_params.kick_direction);
}

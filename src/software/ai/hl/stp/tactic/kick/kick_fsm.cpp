#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"

void KickFSM::updateKick(const Update &event)
{
    //adjust kick speed based on current robot speed
    double projected_robot_speed = event.common.robot.currentState().velocity().dot(event.common.world.ball().velocity().normalize());
    double adjusted_kick_speed = event.control_params.kick_speed_meters_per_second- projected_robot_speed;
    event.common.set_primitive(createMovePrimitive(
        CREATE_MOTION_CONTROL(event.control_params.kick_origin),
        event.control_params.kick_direction, 0.1, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                       adjusted_kick_speed},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
        event.common.robot.robotConstants()));
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

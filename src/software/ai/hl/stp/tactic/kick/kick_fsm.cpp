#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"

void KickFSM::updateKick(const Update &event)
{
    event.common.set_primitive(createKickPrimitive(
        event.control_params.kick_origin, event.control_params.kick_direction,
        event.control_params.kick_speed_meters_per_second,
        event.common.robot.robotConstants(),
        (event.common.robot.position() - event.control_params.kick_origin).length()));
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

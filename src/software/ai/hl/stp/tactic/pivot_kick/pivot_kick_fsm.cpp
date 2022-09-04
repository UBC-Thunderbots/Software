#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"


void PivotKickFSM::getPossessionAndPivot(
    const Update& event, boost::sml::back::process<DribbleFSM::Update> processEvent)
{
    DribbleFSM::ControlParams control_params{
        .dribble_destination       = event.control_params.kick_origin,
        .final_dribble_orientation = event.control_params.kick_direction,
        .allow_excessive_dribbling = false};

    processEvent(DribbleFSM::Update(control_params, event.common));
}

void PivotKickFSM::kickBall(const Update& event)
{
    auto constants = event.common.robot.robotConstants();
    constants.robot_max_ang_speed_rad_per_s =
        constants.robot_max_ang_speed_rad_per_s / 2.0f;
    constants.robot_max_ang_acceleration_rad_per_s_2 =
        constants.robot_max_ang_acceleration_rad_per_s_2 / 2.0f;

    event.common.set_primitive(createMovePrimitive(
        CREATE_MOTION_CONTROL(
            event.control_params.kick_origin +
            Vector::createFromAngle(event.control_params.kick_direction).normalize(0.1)),
        event.control_params.kick_direction, 0, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW, event.control_params.auto_chip_or_kick,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, constants,
        std::optional<double>(), true));
}

bool PivotKickFSM::ballKicked(const Update& event)
{
    if (event.control_params.auto_chip_or_kick.auto_chip_kick_mode ==
        AutoChipOrKickMode::AUTOKICK)
    {
        return event.common.world.ball().hasBallBeenKicked(
            event.control_params.kick_direction);
    }
    else
    {
        // check for separation for chipping since kick angle is not reliable
        return !event.common.robot.isNearDribbler(event.common.world.ball().position(),
                                                  ROBOT_MAX_RADIUS_METERS);
    }
}

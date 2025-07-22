#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"

#include "software/ai/hl/stp/tactic/move_primitive.h"

void KickFSM::updateKick(const Update &event)
{
    Vector direction_to_kick =
        Vector::createFromAngle(event.control_params.kick_direction);
    Point kick_target = event.control_params.kick_origin -
                        direction_to_kick.normalize(DIST_TO_FRONT_OF_ROBOT_METERS - 0.01);

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
    GetBehindBallFSMControlParams control_params{
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

bool KickFSM::shouldRealignWithBall(const Update &event)
{
    const Robot &robot = event.common.robot;

    // First check to see if it's too late to realign with the ball
    if (robot.isNearDribbler(event.control_params.kick_origin, 0.05))
    {
        return false;
    }

    // Check if the robot is already aligned to kick the ball
    return !isRobotReadyToChick(robot, event.control_params.kick_origin,
                                event.control_params.kick_direction);
}

void KickFSM::updateControlParams(const Point &kick_origin,
                                     const Angle &kick_direction,
                                     double kick_speed_meters_per_second)
{
    control_params.kick_origin                  = kick_origin;
    control_params.kick_direction               = kick_direction;
    control_params.kick_speed_meters_per_second = kick_speed_meters_per_second;
}

void KickFSM::updateControlParams(const Point &kick_origin, const Point &kick_target,
                                     double kick_speed_meters_per_second)
{
    updateControlParams(kick_origin, (kick_target - kick_origin).orientation(),
                        kick_speed_meters_per_second);
}

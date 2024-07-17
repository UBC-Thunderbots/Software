#include "software/ai/hl/stp/skill/kick/kick_skill_fsm.h"

#include "software/ai/hl/stp/primitive/move_primitive.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"

void KickSkillFSM::updateKick(const Update &event)
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

void KickSkillFSM::updateGetBehindBall(
    const Update &event,
    boost::sml::back::process<GetBehindBallSkillFSM::Update> processEvent)
{
    GetBehindBallSkillFSM::ControlParams control_params{
        .ball_location   = event.control_params.kick_origin,
        .chick_direction = event.control_params.kick_direction};

    processEvent(GetBehindBallSkillFSM::Update(control_params, event.common));
}

bool KickSkillFSM::ballChicked(const Update &event)
{
    return event.common.world_ptr->ball().hasBallBeenKicked(
        event.control_params.kick_direction);
}

bool KickSkillFSM::shouldRealignWithBall(const Update &event)
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

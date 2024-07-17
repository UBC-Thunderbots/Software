#include "software/ai/hl/stp/skill/chip/chip_skill_fsm.h"

#include "software/ai/hl/stp/primitive/move_primitive.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"

void ChipSkillFSM::updateGetBehindBall(
    const Update &event,
    boost::sml::back::process<GetBehindBallSkillFSM::Update> processEvent)
{
    GetBehindBallSkillFSM::ControlParams control_params{
        .ball_location   = event.control_params.chip_origin,
        .chick_direction = event.control_params.chip_direction};

    // Update the get behind ball fsm
    processEvent(GetBehindBallSkillFSM::Update(control_params, event.common));
}

void ChipSkillFSM::updateChip(const Update &event)
{
    Vector direction_to_chip =
        Vector::createFromAngle(event.control_params.chip_direction);
    Point chip_target = event.control_params.chip_origin -
                        direction_to_chip.normalize(DIST_TO_FRONT_OF_ROBOT_METERS - 0.01);

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, chip_target, event.control_params.chip_direction,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::SAFE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP,
                       event.control_params.chip_distance_meters}));
}

bool ChipSkillFSM::ballChicked(const Update &event)
{
    return event.common.world_ptr->ball().hasBallBeenKicked(
        event.control_params.chip_direction);
}

bool ChipSkillFSM::shouldRealignWithBall(const Update &event)
{
    const Robot &robot = event.common.robot;

    // First check to see if it's too late to realign with the ball
    if (robot.isNearDribbler(event.control_params.chip_origin, 0.05))
    {
        return false;
    }

    // Check if the robot is already aligned to chip the ball
    return !isRobotReadyToChick(robot, event.control_params.chip_origin,
                                event.control_params.chip_direction);
}

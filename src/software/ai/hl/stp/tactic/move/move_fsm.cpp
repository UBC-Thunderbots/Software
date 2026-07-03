#include "software/ai/hl/stp/tactic/move/move_fsm.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/move_primitive.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"

namespace
{
    // The final orientation is considered to be "facing" the enemy goal, and
    // therefore eligible for autokick, if it is within this many degrees of the
    // direction from the robot to the enemy goal center
    const Angle FACING_ENEMY_GOAL_ANGLE_THRESHOLD = Angle::fromDegrees(45);
}  // namespace

MoveFSM::MoveFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : TacticFSM<MoveFSM>(ai_config_ptr)
{
}

void MoveFSM::updateMove(const Update& event)
{
    AutoChipOrKick auto_chip_or_kick = event.control_params.auto_chip_or_kick;

    Angle angle_to_enemy_goal =
        (event.common.world_ptr->field().enemyGoalCenter() -
         event.common.robot.position())
            .orientation();
    if (compareAngles(event.control_params.final_orientation, angle_to_enemy_goal,
                      FACING_ENEMY_GOAL_ANGLE_THRESHOLD))
    {
        auto_chip_or_kick =
            AutoChipOrKick{AutoChipOrKickMode::AUTOKICK, BALL_MAX_SPEED_METERS_PER_SECOND};
    }

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, event.control_params.destination,
        event.control_params.final_orientation,
        event.control_params.max_allowed_speed_mode,
        event.control_params.obstacle_avoidance_mode, event.control_params.dribbler_mode,
        event.control_params.ball_collision_type, auto_chip_or_kick));
}

bool MoveFSM::moveDone(const Update& event)
{
    return robotReachedDestination(event.common.robot, event.control_params.destination,
                                   event.control_params.final_orientation);
}

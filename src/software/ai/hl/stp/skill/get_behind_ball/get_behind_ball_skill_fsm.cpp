#include "software/ai/hl/stp/skill/get_behind_ball/get_behind_ball_skill_fsm.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/hl/stp/primitive/move_primitive.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"

GetBehindBallSkillFSM::GetBehindBallSkillFSM() {}

void GetBehindBallSkillFSM::updateMove(const Update& event)
{
    Vector behind_ball =
        Vector::createFromAngle(event.control_params.chick_direction + Angle::half());
    Point point_behind_ball = event.control_params.ball_location +
                              behind_ball.normalize(2 * ROBOT_MAX_RADIUS_METERS);

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, point_behind_ball, event.control_params.chick_direction,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
}

bool GetBehindBallSkillFSM::behindBall(const Update& event)
{
    return isRobotReadyToChick(event.common.robot, event.control_params.ball_location,
                               event.control_params.chick_direction);
}

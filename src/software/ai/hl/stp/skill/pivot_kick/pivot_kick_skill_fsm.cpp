#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"

#include "software/ai/hl/stp/primitive/move_primitive.h"

void PivotKickSkillFSM::getBallControlAndPivot(
    const Update& event, boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    DribbleSkillFSM::ControlParams control_params{
        .dribble_destination       = event.control_params.kick_origin,
        .final_dribble_orientation = event.control_params.kick_direction,
        .excessive_dribbling_mode  = TbotsProto::ExcessiveDribblingMode::NOT_ALLOWED};

    processEvent(DribbleSkillFSM::Update(control_params, event.common));
}

void PivotKickSkillFSM::setKickStartTime(const Update& event)
{
    kick_start_time_ = event.common.world_ptr->getMostRecentTimestamp();
}

void PivotKickSkillFSM::kickBall(const Update& event)
{
    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, event.control_params.kick_origin,
        event.control_params.kick_direction,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW, event.control_params.auto_chip_or_kick));
}

bool PivotKickSkillFSM::lostBallControl(const Update& event)
{
    const auto& ai_config         = event.common.strategy->getAiConfig();
    const auto& pivot_kick_config = ai_config.pivot_kick_config();

    const Ball& ball = event.common.world_ptr->ball();

    // Ball must be away from dribbler for it to be out of our control
    if (event.common.robot.isNearDribbler(
            ball.position(), pivot_kick_config.lose_ball_control_distance_threshold()))
    {
        return false;
    }

    const Timestamp current_time = event.common.world_ptr->getMostRecentTimestamp();
    const Duration time_since_kick_start = current_time - kick_start_time_;

    const Duration lose_ball_control_time_threshold =
        Duration::fromSeconds(pivot_kick_config.lose_ball_control_time_threshold());
    const double ball_is_kicked_m_per_s_threshold =
        ai_config.ai_parameter_config().ball_is_kicked_m_per_s_threshold();

    // Ball must be still for a period of time since the kick attempt started
    // for it to be considered out of our control; otherwise we might consider
    // kicked balls as out of our control
    if (time_since_kick_start < lose_ball_control_time_threshold ||
        ball.velocity().length() > ball_is_kicked_m_per_s_threshold)
    {
        return false;
    }

    // Ball is out of our control
    return true;
}

bool PivotKickSkillFSM::ballKicked(const Update& event)
{
    if (event.control_params.auto_chip_or_kick.auto_chip_kick_mode ==
        AutoChipOrKickMode::AUTOKICK)
    {
        return event.common.world_ptr->ball().hasBallBeenKicked(
            event.control_params.kick_direction);
    }
    else
    {
        // check for separation for chipping since kick angle is not reliable
        return !event.common.robot.isNearDribbler(
            event.common.world_ptr->ball().position(), ROBOT_MAX_RADIUS_METERS);
    }
}

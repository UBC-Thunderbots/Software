#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"

#include "software/ai/hl/stp/primitive/move_primitive.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"

Point DribbleSkillFSM::robotPositionToFaceBall(const Point &ball_position,
                                               const Angle &face_ball_angle,
                                               double additional_offset)
{
    return ball_position - Vector::createFromAngle(face_ball_angle)
                               .normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                          BALL_MAX_RADIUS_METERS + additional_offset);
}

Point DribbleSkillFSM::findInterceptionPoint(const Robot &robot, const Ball &ball,
                                             const Field &field)
{
    static constexpr double BALL_MOVING_SLOW_SPEED_THRESHOLD   = 0.3;
    static constexpr double INTERCEPT_POSITION_SEARCH_INTERVAL = 0.1;
    if (ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD)
    {
        auto face_ball_vector = (ball.position() - robot.position());
        auto point_in_front_of_ball =
            robotPositionToFaceBall(ball.position(), face_ball_vector.orientation());
        return point_in_front_of_ball;
    }
    Point intercept_position = ball.position();
    while (contains(field.fieldLines(), intercept_position))
    {
        Duration ball_time_to_position = Duration::fromSeconds(
            distance(intercept_position, ball.position()) / ball.velocity().length());
        Duration robot_time_to_pos = robot.getTimeToPosition(intercept_position);

        if (robot_time_to_pos < ball_time_to_position)
        {
            break;
        }
        intercept_position +=
            ball.velocity().normalize(INTERCEPT_POSITION_SEARCH_INTERVAL);
    }
    return intercept_position;
}

Point DribbleSkillFSM::getDribbleBallDestination(const Point &ball_position,
                                                 std::optional<Point> dribble_destination)
{
    // Default is the current ball position
    Point target_dest = ball_position;
    if (dribble_destination)
    {
        target_dest = dribble_destination.value();
    }
    return target_dest;
}

Angle DribbleSkillFSM::getFinalDribbleOrientation(
    const Point &ball_position, const Point &robot_position,
    std::optional<Angle> final_dribble_orientation)
{
    // Default is face ball direction
    Angle target_orientation = (ball_position - robot_position).orientation();
    if (final_dribble_orientation)
    {
        target_orientation = final_dribble_orientation.value();
    }
    return target_orientation;
}

std::tuple<Point, Angle> DribbleSkillFSM::calculateNextDribbleDestinationAndOrientation(
    const Ball &ball, const Robot &robot, std::optional<Point> dribble_destination_opt,
    std::optional<Angle> final_dribble_orientation_opt)
{
    Point dribble_destination =
        getDribbleBallDestination(ball.position(), dribble_destination_opt);

    // Default destination and orientation assume ball is at the destination
    // pivot to final face ball destination
    Angle target_orientation = getFinalDribbleOrientation(
        ball.position(), robot.position(), final_dribble_orientation_opt);
    Point target_destination =
        robotPositionToFaceBall(dribble_destination, target_orientation);

    return std::make_tuple(target_destination, target_orientation);
}

void DribbleSkillFSM::getPossession(const Update &event)
{
    auto ball_position = event.common.world_ptr->ball().position();
    auto face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();
    Point intercept_position =
        findInterceptionPoint(event.common.robot, event.common.world_ptr->ball(),
                              event.common.world_ptr->field()) +
        Vector::createFromAngle(face_ball_orientation).normalize(0.05);

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, intercept_position, face_ball_orientation,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
}

void DribbleSkillFSM::dribble(const Update &event)
{
    auto [target_destination, target_orientation] =
        calculateNextDribbleDestinationAndOrientation(
            event.common.world_ptr->ball(), event.common.robot,
            event.control_params.dribble_destination,
            event.control_params.final_dribble_orientation);

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, target_destination, target_orientation,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
}

void DribbleSkillFSM::loseBall(const Update &event)
{
    const TbotsProto::DribbleSkillConfig &dribble_skill_config =
        event.common.strategy->getAiConfig().dribble_skill_config();

    Point ball_position = event.common.world_ptr->ball().position();
    auto face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();
    Point away_from_ball_position = robotPositionToFaceBall(
        ball_position, face_ball_orientation,
        dribble_skill_config.lose_ball_possession_threshold() * 2);

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, away_from_ball_position, face_ball_orientation,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID,
        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK, 0.5}));
}

void DribbleSkillFSM::startDribble(const Update &event)
{
    // update continuous_dribbling_start_point once we start dribbling
    continuous_dribbling_start_point = event.common.world_ptr->ball().position();
    dribble(event);
}

bool DribbleSkillFSM::havePossession(const Update &event)
{
    return event.common.robot.isNearDribbler(event.common.world_ptr->ball().position());
}

bool DribbleSkillFSM::lostPossession(const Update &event)
{
    const TbotsProto::DribbleSkillConfig &dribble_skill_config =
        event.common.strategy->getAiConfig().dribble_skill_config();

    return !event.common.robot.isNearDribbler(
        // avoid cases where ball is exactly on the edge of the robot
        event.common.world_ptr->ball().position(),
        dribble_skill_config.lose_ball_possession_threshold());
};

bool DribbleSkillFSM::dribblingDone(const Update &event)
{
    const TbotsProto::DribbleSkillConfig &dribble_skill_config =
        event.common.strategy->getAiConfig().dribble_skill_config();

    return comparePoints(
               event.common.world_ptr->ball().position(),
               getDribbleBallDestination(event.common.world_ptr->ball().position(),
                                         event.control_params.dribble_destination),
               dribble_skill_config.ball_close_to_dest_threshold()) &&
           compareAngles(
               event.common.robot.orientation(),
               getFinalDribbleOrientation(event.common.world_ptr->ball().position(),
                                          event.common.robot.position(),
                                          event.control_params.final_dribble_orientation),
               Angle::fromDegrees(
                   dribble_skill_config.final_destination_close_threshold())) &&
           havePossession(event) &&
           robotStopped(event.common.robot,
                        dribble_skill_config.robot_dribbling_done_speed());
}

bool DribbleSkillFSM::shouldLoseBall(const Update &event)
{
    const TbotsProto::DribbleSkillConfig &dribble_skill_config =
        event.common.strategy->getAiConfig().dribble_skill_config();

    Point ball_position = event.common.world_ptr->ball().position();
    return (!event.control_params.allow_excessive_dribbling &&
            !comparePoints(ball_position, continuous_dribbling_start_point,
                           dribble_skill_config.max_continuous_dribbling_distance()));
}

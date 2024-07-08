#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"

#include "software/ai/hl/stp/primitive/move_primitive.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"

Point DribbleSkillFSM::robotPositionToFaceBall(const Point &ball_position,
                                               const Angle &face_ball_angle,
                                               double additional_offset)
{
    return ball_position - Vector::createFromAngle(face_ball_angle)
                               .normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                          BALL_MAX_RADIUS_METERS + additional_offset);
}

Point DribbleSkillFSM::findInterceptionPoint(
    const Robot &robot, const Ball &ball, const Field &field,
    const TbotsProto::DribbleConfig &dribble_config)
{
    static constexpr double BALL_MOVING_SLOW_SPEED_THRESHOLD   = 0.3;
    static constexpr double INTERCEPT_POSITION_SEARCH_INTERVAL = 0.1;

    if (ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD)
    {
        auto face_ball_vector = (ball.position() - robot.position());

        // Draw a long polygon from the center of robot forward, with the width of the
        // dribbler that the ball should be within before we try to dribble into it.
        float dribbler_width      = robot.robotConstants().dribbler_width_meters;
        Vector robot_front_vector = Vector::createFromAngle(robot.orientation());
        Polygon infront_of_dribbler_polygon = Polygon::fromSegment(
            Segment(robot.position(),
                    robot.position() + robot_front_vector.normalize(20)),
            0.0, dribbler_width / 2.0);

        bool dribbler_aligned_with_ball =
            contains(infront_of_dribbler_polygon, ball.position());
        bool robot_turning_too_fast =
            robot.angularVelocity().toDegrees() >
            dribble_config.max_robot_angular_vel_when_getting_possession_deg_per_s();

        double offset_to_ball = 0.0;
        if (!dribbler_aligned_with_ball || robot_turning_too_fast)
        {
            // The ball is not infront of the robot, or the robot is turning too fast
            // so add some additional offset to the ball destination, so we don't bump
            // into it.
            offset_to_ball = dribble_config.offset_to_ball_when_not_aligned_meters();
        }

        auto point_in_front_of_ball = robotPositionToFaceBall(
            ball.position(), face_ball_vector.orientation(), offset_to_ball);
        return point_in_front_of_ball;
    }

    Point intercept_position = ball.position();
    while (contains(field.fieldLines(), intercept_position))
    {
        Duration ball_time_to_position = Duration::fromSeconds(
            distance(intercept_position, ball.position()) / ball.velocity().length());
        Duration robot_time_to_pos = robot.getTimeToPosition(intercept_position);

        // Give the robot some slack time to intercept the ball.
        // The slack time is reduced as we get closer to the ball and have
        // a better chance of intercepting it.
        Duration slack_time_sec = std::min(
            ball_time_to_position,
            Duration::fromSeconds(dribble_config.max_ball_interception_slack_time_sec()));

        if (robot_time_to_pos < (ball_time_to_position + slack_time_sec))
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

void DribbleSkillFSM::getBallControl(const Update &event)
{
    auto ball_position = event.common.world_ptr->ball().position();
    auto face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();
    Point intercept_position =
        findInterceptionPoint(event.common.robot, event.common.world_ptr->ball(),
                              event.common.world_ptr->field(),
                              event.common.strategy->getAiConfig().dribble_config()) +
        Vector::createFromAngle(face_ball_orientation).normalize(0.05);

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, intercept_position, face_ball_orientation,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE,
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
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE,
        TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
}

void DribbleSkillFSM::loseBall(const Update &event)
{
    const TbotsProto::DribbleConfig &dribble_config =
        event.common.strategy->getAiConfig().dribble_config();

    Point ball_position = event.common.world_ptr->ball().position();
    auto face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();
    Point away_from_ball_position =
        robotPositionToFaceBall(ball_position, face_ball_orientation,
                                dribble_config.lose_ball_control_threshold() * 2);

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, away_from_ball_position, face_ball_orientation,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID,
        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK, 0.5}));
}

bool DribbleSkillFSM::haveBallControl(const Update &event)
{
    return event.common.robot.isNearDribbler(event.common.world_ptr->ball().position());
}

bool DribbleSkillFSM::lostBallControl(const Update &event)
{
    const TbotsProto::DribbleConfig &dribble_config =
        event.common.strategy->getAiConfig().dribble_config();

    return !event.common.robot.isNearDribbler(
        // avoid cases where ball is exactly on the edge of the robot
        event.common.world_ptr->ball().position(),
        dribble_config.lose_ball_control_threshold());
};

bool DribbleSkillFSM::dribblingDone(const Update &event)
{
    const TbotsProto::DribbleConfig &dribble_config =
        event.common.strategy->getAiConfig().dribble_config();

    return comparePoints(
               event.common.world_ptr->ball().position(),
               getDribbleBallDestination(event.common.world_ptr->ball().position(),
                                         event.control_params.dribble_destination),
               dribble_config.ball_close_to_dest_threshold()) &&
           compareAngles(
               event.common.robot.orientation(),
               getFinalDribbleOrientation(event.common.world_ptr->ball().position(),
                                          event.common.robot.position(),
                                          event.control_params.final_dribble_orientation),
               Angle::fromDegrees(
                   dribble_config.final_destination_close_threshold_deg())) &&
           !lostBallControl(event) &&
           robotStopped(event.common.robot, dribble_config.robot_dribbling_done_speed());
}

bool DribbleSkillFSM::shouldLoseBall(const Update &event)
{
    const TbotsProto::DribbleConfig &dribble_config =
        event.common.strategy->getAiConfig().dribble_config();

    return !event.control_params.allow_excessive_dribbling &&
           event.common.world_ptr->getDistanceDribbledByFriendlyTeam() >=
               dribble_config.max_continuous_dribbling_distance();
}

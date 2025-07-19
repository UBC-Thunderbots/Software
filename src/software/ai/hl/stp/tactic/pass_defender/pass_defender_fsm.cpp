#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_fsm.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/hl/stp/tactic/move_primitive.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/logger/logger.h"

bool PassDefenderFSM::passStarted(const Update& event)
{
    auto ball_position = event.common.world_ptr->ball().position();
    Vector ball_receiver_point_vector(
        event.control_params.position_to_block_from.x() - ball_position.x(),
        event.control_params.position_to_block_from.y() - ball_position.y());

    bool pass_started = event.common.world_ptr->ball().hasBallBeenKicked(
        ball_receiver_point_vector.orientation(), MIN_PASS_SPEED,
        MAX_PASS_ANGLE_DIFFERENCE);

    if (pass_started)
    {
        // We want to keep track of the initial trajectory of the pass
        // so that we can later tell whether the ball strays from
        // this trajectory
        pass_orientation = event.common.world_ptr->ball().velocity().orientation();
    }

    return pass_started;
}

bool PassDefenderFSM::ballDeflected(const Update& event)
{
    auto orientation_difference =
        event.common.world_ptr->ball().velocity().orientation().minDiff(pass_orientation);

    // If the ball strays from the initial trajectory of the pass,
    // it was likely deflected off course by another robot or chipped
    // away by the pass defender
    return orientation_difference.abs() > MIN_DEFLECTION_ANGLE;
}

void PassDefenderFSM::blockPass(const Update& event)
{
    auto position_to_block_from = event.control_params.position_to_block_from;
    auto ball_position          = event.common.world_ptr->ball().position();
    auto face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();

    // Face the ball and move to position_to_block_from, which should be a location
    // on the field that blocks a passing lane between two enemy robots
    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, position_to_block_from, face_ball_orientation,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
}

void PassDefenderFSM::interceptBall(const Update& event)
{
    auto ball           = event.common.world_ptr->ball();
    auto robot_position = event.common.robot.position();

    if ((ball.position() - robot_position).length() >
        BALL_TO_FRONT_OF_ROBOT_DISTANCE_WHEN_DRIBBLING)
    {
        Point intercept_position = ball.position();
        if (ball.velocity().length() != 0)
        {
            // Find the closest point on the line of the ball's current trajectory
            // that the defender can move to and intercept the pass
            intercept_position = closestPoint(
                robot_position, Line(ball.position(), ball.position() + ball.velocity()));

            // Here we are using v_f^2=v_0^2+2*a*d to calculate the final
            // speed and after that the average speed
            double ball_vel = (event.common.world_ptr->ball().velocity().length());
            double ball_dis =
                (event.common.world_ptr->ball().position() - intercept_position).length();
            double final_vel =
                sqrt(ball_vel * ball_vel +
                     2 * ball_dis *
                         BALL_ROLLING_FRICTION_DECELERATION_METERS_PER_SECOND_SQUARED);
            double avg_vel               = (final_vel + ball_vel) / 2.0;
            Duration ball_intercept_time = Duration::fromSeconds(
                ball_dis / (std::max(std::numeric_limits<double>::epsilon(), avg_vel)));

            // Here we check if we can make it in time to the position and stop in time
            // otherwise we will attempt to overshoot the position and intercept it
            if (event.common.robot.getTimeToPosition(intercept_position) >
                ball_intercept_time)
            {
                Point new_destination = intercept_position;
                double final_speed    = DEFENDER_STEP_SPEED_M_PER_S;
                bool finished         = false;
                double max_speed =
                    event.common.robot.robotConstants().robot_max_speed_m_per_s;
                double max_acc =
                    event.common.robot.robotConstants().robot_max_acceleration_m_per_s_2;
                Point possible_intercept_pos = intercept_position;

                while (!finished)
                {
                    Vector final_velocity =
                        (new_destination - event.common.robot.position())
                            .normalize(final_speed);

                    // What's happening here is we are using v_o^2=2*d*a to determine the
                    // extra distance the defender will be forced to travel if they
                    // intercept the ball with final_speed and then immediately decelerate
                    double extra_length = (final_speed * final_speed) / (2.0 * max_acc);
                    new_destination =
                        possible_intercept_pos + final_velocity.normalize(extra_length);
                    if (!event.common.world_ptr->field().pointInFriendlyDefenseArea(
                            new_destination) &&
                        contains(event.common.world_ptr->field().fieldBoundary(),
                                 new_destination))
                    {
                        if (event.common.robot.getTimeToPosition(possible_intercept_pos,
                                                                 final_velocity) <
                            ball_intercept_time)
                        {
                            // If we found that the robot can make it before the ball we
                            // consider the while loop finished
                            intercept_position = new_destination;
                            finished           = true;
                        }
                    }
                    else
                    {
                        finished = true;
                    }
                    final_speed += DEFENDER_STEP_SPEED_M_PER_S;
                    if (final_speed > max_speed)
                    {
                        // Couldn't find a speed possible to intercept
                        finished = true;
                    }
                }
            }
        }

        auto face_ball_orientation = (ball.position() - robot_position).orientation();

        // Move to intercept the pass by positioning defender in front of the
        // ball's current trajectory
        event.common.set_primitive(std::make_unique<MovePrimitive>(
            event.common.robot, intercept_position, face_ball_orientation,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE,
            TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
        return;
    }

    // The ball is likely above the robot
    // to avoid dividing by 0
    Angle face_ball_orientation;
    if ((ball.position() - robot_position).length() == 0)
    {
        face_ball_orientation = event.common.robot.orientation();
    }
    else
    {
        face_ball_orientation = (ball.position() - robot_position).orientation();
    }

    // backup by the length of the ball
    Point backup_position =
        robot_position + ball.velocity().normalize(ROBOT_MAX_RADIUS_METERS);
    event.common.set_primitive(std::make_shared<MovePrimitive>(
        event.common.robot, backup_position, face_ball_orientation,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE,
        TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
}

bool PassDefenderFSM::ballNearbyWithoutThreat(const Update& event)
{
    return DefenderFSMBase::ballNearbyWithoutThreat(
        event.common.world_ptr, event.common.robot, event.control_params.ball_steal_mode,
        pass_defender_config.defender_steal_config());
}

void PassDefenderFSM::prepareGetPossession(
    const Update& event, boost::sml::back::process<DribbleFSM::Update> processEvent)
{
    DefenderFSMBase::prepareGetPossession(event.common, processEvent);
}

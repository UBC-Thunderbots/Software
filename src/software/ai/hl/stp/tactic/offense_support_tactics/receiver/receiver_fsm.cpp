#include "software/ai/hl/stp/tactic/offense_support_tactics/receiver/receiver_fsm.h"

#include "software/ai/hl/stp/primitive/move_primitive.h"
#include "software/geom/algorithms/convex_angle.h"

Angle ReceiverFSM::getOneTouchShotDirection(const Ray& shot, const Ball& ball)
{
    Vector shot_vector = shot.toUnitVector();
    Angle shot_dir     = shot.getDirection();

    Vector ball_vel    = ball.velocity();
    Vector lateral_vel = ball_vel.project(shot_vector.perpendicular());

    // The lateral speed is roughly a measure of the lateral velocity we need to
    // "cancel out" in order for our shot to go in the expected direction.
    // The scaling factor of 0.3 is a magic number that was carried over from the old
    // code. It seems to work well on the field.
    // TODO (#2570): tune this
    double lateral_speed = 0.3 * lateral_vel.length();

    // This kick speed is based off of the value used in the firmware `MovePrimitive`
    // when autokick is enabled
    double kick_speed = BALL_MAX_SPEED_METERS_PER_SECOND - 1;
    Angle shot_offset = Angle::asin(lateral_speed / kick_speed);

    // check which direction the ball is going in so we can decide which direction to
    // apply the offset in
    if (lateral_vel.dot(shot_vector.rotate(Angle::quarter())) > 0)
    {
        // need to go clockwise
        shot_offset = -shot_offset;
    }
    return shot_dir + shot_offset;
}

Shot ReceiverFSM::getOneTouchShotPositionAndOrientation(const Robot& robot,
                                                        const Ball& ball,
                                                        const Point& best_shot_target)
{
    double dist_to_ball_in_dribbler =
        DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS;
    Point ball_contact_point =
        robot.position() +
        Vector::createFromAngle(robot.orientation()).normalize(dist_to_ball_in_dribbler);

    // Find the closest point to the ball contact point on the ball's trajectory
    Point closest_ball_pos = ball.position();
    if (ball.velocity().length() >= BALL_MIN_MOVEMENT_SPEED)
    {
        closest_ball_pos = closestPoint(
            ball_contact_point, Line(ball.position(), ball.position() + ball.velocity()));
    }
    Ray shot(closest_ball_pos, best_shot_target - closest_ball_pos);

    Angle ideal_orientation      = getOneTouchShotDirection(shot, ball);
    Vector ideal_orientation_vec = Vector::createFromAngle(ideal_orientation);

    // The best position is determined such that the robot stays in the ideal
    // orientation, but moves the shortest distance possible to put its contact
    // point in the ball's path.
    Point ideal_position =
        closest_ball_pos - ideal_orientation_vec.normalize(dist_to_ball_in_dribbler);

    return Shot(ideal_position, ideal_orientation);
}

std::optional<Shot> ReceiverFSM::findFeasibleShot(const WorldPtr& world_ptr,
                                                  const Robot& assigned_robot)
{
    // Check if we can shoot on the enemy goal from the receiver position
    std::optional<Shot> best_shot_opt = calcBestShotOnGoal(
        world_ptr->field(), world_ptr->friendlyTeam(), world_ptr->enemyTeam(),
        assigned_robot.position(), TeamType::ENEMY, {assigned_robot});

    // The percentage of open net the robot would shoot on
    if (best_shot_opt)
    {
        // Vector from the ball to the robot
        Vector robot_to_ball = world_ptr->ball().position() - assigned_robot.position();

        // The angle the robot will have to deflect the ball to shoot
        Angle abs_angle_between_pass_and_shot_vectors;

        Vector robot_to_shot_target =
            best_shot_opt.value().getPointToShootAt() - assigned_robot.position();
        abs_angle_between_pass_and_shot_vectors =
            convexAngle(robot_to_ball, robot_to_shot_target);

        Angle goal_angle = convexAngle(world_ptr->field().friendlyGoalpostPos(),
                                       assigned_robot.position(),
                                       world_ptr->field().friendlyGoalpostNeg());

        double net_percent_open =
            best_shot_opt.value().getOpenAngle().toDegrees() / goal_angle.toDegrees();

        // If we have a shot with a sufficiently large enough opening, and the
        // deflection angle that is reasonable, we should one-touch kick the ball
        // towards the enemy net
        if (net_percent_open > MIN_SHOT_NET_PERCENT_OPEN &&
            abs_angle_between_pass_and_shot_vectors < MAX_DEFLECTION_FOR_ONE_TOUCH_SHOT)
        {
            return best_shot_opt;
        }
    }

    return std::nullopt;
}

bool ReceiverFSM::onetouchPossible(const Update& event)
{
    return !event.control_params.disable_one_touch_shot &&
           (findFeasibleShot(event.common.world_ptr, event.common.robot) != std::nullopt);
}

void ReceiverFSM::updateOnetouch(const Update& event)
{
    auto best_shot = findFeasibleShot(event.common.world_ptr, event.common.robot);
    auto one_touch = getOneTouchShotPositionAndOrientation(
        event.common.robot, event.common.world_ptr->ball(),
        best_shot->getPointToShootAt());

    if (best_shot && event.control_params.pass)
    {
        event.common.set_primitive(std::make_unique<MovePrimitive>(
            event.common.robot, one_touch.getPointToShootAt(), one_touch.getOpenAngle(),
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::DribblerMode::OFF, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                           BALL_MAX_SPEED_METERS_PER_SECOND}));
    }
    else
    {
        event.common.set_primitive(std::make_unique<StopPrimitive>());
    }
}

void ReceiverFSM::updateReceive(const Update& event)
{
    if (event.control_params.pass)
    {
        event.common.set_primitive(std::make_unique<MovePrimitive>(
            event.common.robot, event.control_params.pass->receiverPoint(),
            event.control_params.pass->receiverOrientation(),
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
    }
}

void ReceiverFSM::adjustReceive(const Update& event)
{
    auto ball      = event.common.world_ptr->ball();
    auto robot_pos = event.common.robot.position();

    if ((ball.position() - robot_pos).length() >
        BALL_TO_FRONT_OF_ROBOT_DISTANCE_WHEN_DRIBBLING)
    {
        Point ball_receive_pos = ball.position();

        if (ball.velocity().length() > MIN_PASS_START_SPEED)
        {
            ball_receive_pos = closestPoint(
                robot_pos, Line(ball.position(), ball.position() + ball.velocity()));
        }

        Angle ball_receive_orientation = (ball.position() - robot_pos).orientation();

        event.common.set_primitive(std::make_unique<MovePrimitive>(
            event.common.robot, ball_receive_pos, ball_receive_orientation,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
    }
}

bool ReceiverFSM::passStarted(const Update& event)
{
    return event.common.world_ptr->ball().hasBallBeenKicked(
        event.control_params.pass->passerOrientation());
}

bool ReceiverFSM::passFinished(const Update& event)
{
    // We tolerate imperfect passes that hit the edges of the robot,
    // so that we can quickly transition out and grab the ball.
    return event.common.robot.isNearDribbler(event.common.world_ptr->ball().position());
}

bool ReceiverFSM::strayPass(const Update& event)
{
    auto ball_position = event.common.world_ptr->ball().position();

    Vector ball_receiver_point_vector(
        event.control_params.pass->receiverPoint().x() - ball_position.x(),
        event.control_params.pass->receiverPoint().y() - ball_position.y());

    auto orientation_difference =
        event.common.world_ptr->ball().velocity().orientation() -
        ball_receiver_point_vector.orientation();

    // if pass has strayed far from its intended destination (ex it was deflected)
    // we consider the pass finished
    bool stray_pass =
        event.common.world_ptr->ball().velocity().length() > MIN_STRAY_PASS_SPEED &&
        orientation_difference > MIN_STRAY_PASS_ANGLE;

    return stray_pass;
}

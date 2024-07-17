#include "software/ai/hl/stp/tactic/receiver/receiver_fsm.h"

#include "software/ai/evaluation/intercept.h"
#include "software/ai/hl/stp/primitive/move_primitive.h"
#include "software/geom/algorithms/convex_angle.h"

ReceiverFSM::ReceiverFSM(std::shared_ptr<Strategy> strategy) : strategy_(strategy) {}

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
    if (ball.velocity().length() >= BALL_MIN_MOVEMENT_SPEED_M_PER_SEC)
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

    return Shot(closest_ball_pos, ideal_position, ideal_orientation);
}

std::optional<Shot> ReceiverFSM::findFeasibleShot(const World& world,
                                                  const Robot& assigned_robot)
{
    // Check if we can shoot on the enemy goal from the receiver position
    std::optional<Shot> best_shot_opt =
        calcBestShotOnGoal(world.field(), world.friendlyTeam(), world.enemyTeam(),
                           assigned_robot.position(), TeamType::ENEMY, {assigned_robot});

    // The percentage of open net the robot would shoot on
    if (best_shot_opt)
    {
        TbotsProto::ReceiverTacticConfig receiver_tactic_config =
            strategy_->getAiConfig().receiver_tactic_config();

        Vector robot_to_ball = world.ball().position() - assigned_robot.position();

        // The angle the robot will have to deflect the ball to shoot
        Vector robot_to_shot_target =
            best_shot_opt.value().getPointToShootAt() - assigned_robot.position();
        double abs_angle_deg_between_pass_and_shot_vectors =
            convexAngle(robot_to_ball, robot_to_shot_target).toDegrees();

        double shot_open_angle = best_shot_opt.value().getOpenAngle().toDegrees();
        double min_one_touch_open_angle =
            receiver_tactic_config.min_open_angle_for_one_touch_deg();
        double max_one_touch_deflection_angle =
            receiver_tactic_config.max_deflection_for_one_touch_deg();

        // If we have a shot with a sufficiently large enough opening, and the
        // deflection angle that is reasonable, we should one-touch kick the ball
        // towards the enemy net
        if (shot_open_angle > min_one_touch_open_angle &&
            abs_angle_deg_between_pass_and_shot_vectors < max_one_touch_deflection_angle)
        {
            return best_shot_opt;
        }
    }

    return std::nullopt;
}

bool ReceiverFSM::onetouchPossible(const Update& event)
{
    return event.control_params.enable_one_touch_shot &&
           strategy_->getAiConfig().receiver_tactic_config().enable_one_touch_kick() &&
           (findFeasibleShot(*event.common.world_ptr, event.common.robot) !=
            std::nullopt);
}

void ReceiverFSM::updateOnetouch(const Update& event)
{
    auto best_shot = findFeasibleShot(*event.common.world_ptr, event.common.robot);

    if (best_shot.has_value() && event.control_params.receiving_position)
    {
        auto one_touch = getOneTouchShotPositionAndOrientation(
            event.common.robot, event.common.world_ptr->ball(),
            best_shot->getPointToShootAt());

        event.common.set_primitive(std::make_unique<MovePrimitive>(
            event.common.robot, one_touch.getPointToShootAt(), one_touch.getOpenAngle(),
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE, TbotsProto::DribblerMode::OFF,
            TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                           BALL_MAX_SPEED_METERS_PER_SECOND - 0.5}));
    }
    else
    {
        event.common.set_primitive(std::make_unique<StopPrimitive>());
    }
}

void ReceiverFSM::updateReceive(const Update& event)
{
    if (event.control_params.receiving_position)
    {
        const Ball& ball               = event.common.world_ptr->ball();
        const Point receiving_position = event.control_params.receiving_position.value();
        const Angle receiver_orientation =
            (ball.position() - receiving_position).orientation();

        event.common.set_primitive(std::make_unique<MovePrimitive>(
            event.common.robot, receiving_position, receiver_orientation,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE,
            TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
    }
}

void ReceiverFSM::adjustReceive(const Update& event)
{
    const Ball& ball   = event.common.world_ptr->ball();
    const Robot& robot = event.common.robot;

    Angle face_ball_orientation = (ball.position() - robot.position()).orientation();

    Point intercept_position = closestPoint(
        robot.position(), Line(ball.position(), ball.position() + ball.velocity()));

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, intercept_position, face_ball_orientation,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE,
        TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
}

void ReceiverFSM::retrieveBall(
    const Update& event, boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    const Ball& ball   = event.common.world_ptr->ball();
    const Robot& robot = event.common.robot;

    DribbleSkillFSM::ControlParams control_params{
        .dribble_destination       = ball.position(),
        .final_dribble_orientation = (ball.position() - robot.position()).orientation(),
        .excessive_dribbling_mode  = TbotsProto::ExcessiveDribblingMode::LOSE_BALL,
    };

    processEvent(DribbleSkillFSM::Update(
        control_params, SkillUpdate(event.common.robot, event.common.world_ptr, strategy_,
                                    event.common.set_primitive)));
}

bool ReceiverFSM::passStarted(const Update& event)
{
    const Ball& ball               = event.common.world_ptr->ball();
    const Point receiving_position = event.control_params.receiving_position.value();

    return ball.hasBallBeenKicked((receiving_position - ball.position()).orientation());
}

bool ReceiverFSM::passReceived(const Update& event)
{
    return event.common.robot.isNearDribbler(event.common.world_ptr->ball().position());
}

bool ReceiverFSM::passReceivedByTeammate(const Update& event)
{
    const double ball_speed = event.common.world_ptr->ball().velocity().length();
    const double ball_is_kicked_threshold =
        strategy_->getAiConfig().ai_parameter_config().ball_is_kicked_m_per_s_threshold();

    if (ball_speed > ball_is_kicked_threshold)
    {
        return false;
    }

    const auto friendly_robots =
        event.common.world_ptr->friendlyTeam().getAllRobotsExcept({event.common.robot});

    return std::any_of(
        friendly_robots.begin(), friendly_robots.end(), [&](const Robot& robot) {
            return robot.isNearDribbler(event.common.world_ptr->ball().position());
        });
}

bool ReceiverFSM::strayPass(const Update& event)
{
    auto ball_position = event.common.world_ptr->ball().position();

    Vector ball_receiver_point_vector(
        event.control_params.receiving_position->x() - ball_position.x(),
        event.control_params.receiving_position->y() - ball_position.y());

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

bool ReceiverFSM::slowPass(const Update& event)
{
    return event.common.world_ptr->ball().velocity().length() <= MIN_STRAY_PASS_SPEED;
}

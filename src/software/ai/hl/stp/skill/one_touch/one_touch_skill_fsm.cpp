#include "software/ai/hl/stp/skill/one_touch/one_touch_skill_fsm.h"

#include "software/geom/algorithms/contains.h"
#include "software/math/math_functions.h"

Angle OneTouchSkillFSM::getOneTouchShotDirection(const Ray& shot, const Ball& ball)
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

Shot OneTouchSkillFSM::getOneTouchShotPositionAndOrientation(const Robot& robot,
                                                             const Ball& ball,
                                                             const Field &field,
                                                             const Point& best_shot_target)
{
    double dist_to_ball_in_dribbler = DIST_TO_FRONT_OF_ROBOT_METERS;
    Point ball_contact_point =
        robot.position() +
        Vector::createFromAngle(robot.orientation()).normalize(dist_to_ball_in_dribbler);

    // Find the closest point to the ball contact point on the ball's trajectory
    Point closest_ball_pos = ball.position();
    if (ball.velocity().length() >= BALL_MIN_MOVEMENT_SPEED)
    {
        closest_ball_pos = closestPoint(
            ball_contact_point, Line(ball.position(), ball.position() + ball.velocity()));
        if ((closest_ball_pos - ball.position()).orientation() != ball.velocity().orientation())
        {
            closest_ball_pos = ball.position();
        }
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

bool OneTouchSkillFSM::foundShot(const Update& event)
{
    best_shot_ = calcBestShotOnGoal(event.common.world_ptr->field(), 
                                    event.common.world_ptr->friendlyTeam(), 
                                    event.common.world_ptr->enemyTeam(), 
                                    event.common.robot.position(), 
                                    TeamType::ENEMY, {event.common.robot});
    return best_shot_.has_value();
}

bool OneTouchSkillFSM::ballKicked(const Update& event)
{
    return event.common.world_ptr->ball().hasBallBeenKicked(
        (best_shot_->getPointToShootAt() - event.common.world_ptr->ball().position()).orientation());
}

void OneTouchSkillFSM::updateOneTouch(const Update& event)
{
    auto one_touch = getOneTouchShotPositionAndOrientation(
        event.common.robot, event.common.world_ptr->ball(),
        event.common.world_ptr->field(), best_shot_->getPointToShootAt());

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, one_touch.getPointToShootAt(), one_touch.getOpenAngle(),
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK, BALL_MAX_SPEED_METERS_PER_SECOND}));
}

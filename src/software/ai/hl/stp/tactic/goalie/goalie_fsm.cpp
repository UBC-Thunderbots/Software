#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"
#include "software/ai/evaluation/intercept.h"
#include "software/ai/evaluation/find_open_areas.h"

Point GoalieFSM::getGoaliePositionToBlock(const RobotConstants_t robot_constants, const Team &friendly_team,
    const Ball &ball, const Field &field)
{

    //strategy: we assign the robot to cover the largest uncovered part of the goal, starting from the center of the goal
    // to find this , we sweep to the positive and negative sides of the goals to find where friendly robots may be covering
    // parts of the goal.
    double INTERSECTION_INCREMENT_INTERVAL = BALL_MAX_RADIUS_METERS;

    Segment ball_to_pos_goal = Segment(ball.position(), field.friendlyGoalpostPos());
    Segment ball_to_neg_goal = Segment(ball.position(), field.friendlyGoalpostNeg());

    Segment positive_sweep = Segment(ball.position(), field.friendlyGoalCenter());
    Segment negative_sweep = Segment(ball.position(), field.friendlyGoalCenter());

    bool pos_intersection_found = false;
    bool neg_intersection_found = false;

    auto robots = friendly_team.getAllRobotsExceptGoalie();

    while((positive_sweep.getEnd().y() < ball_to_pos_goal.getEnd().y() && negative_sweep.getEnd().y() > ball_to_neg_goal.getEnd().y() )){

        if (!pos_intersection_found){

            //check if the current positive sweep will intersect with a friendly robot
            pos_intersection_found = std::any_of(robots.begin(), robots.end(), [=](Robot robot){
                Circle robot_circle = Circle(robot.position(), robot_constants.robot_radius_m);
                return intersects(robot_circle, positive_sweep);
            });

            //if no friendly robot intersects this path to the goal, then we increase the sweep that the goalie needs to cover
            if(!pos_intersection_found){
                Point new_end = Point(positive_sweep.getEnd().x(), positive_sweep.getEnd().y() + INTERSECTION_INCREMENT_INTERVAL);
                positive_sweep.setEnd(new_end);
            }
        }

        if(!neg_intersection_found){
            neg_intersection_found = std::any_of(robots.begin(), robots.end(), [=](Robot robot){
                Circle robot_circle = Circle(robot.position(), robot_constants.robot_radius_m);
                return intersects(robot_circle, negative_sweep);
            });

            if(!neg_intersection_found){
                Point new_end = Point(negative_sweep.getEnd().x(), negative_sweep.getEnd().y() - INTERSECTION_INCREMENT_INTERVAL);
                negative_sweep.setEnd(new_end);
            }
        }

        if(neg_intersection_found && pos_intersection_found){
            break;
        }
    }

    Angle angle_to_cover = acuteAngle(negative_sweep.getEnd(), ball.position(),
                                      positive_sweep.getEnd());

    Point clamped_goalie_pos = field.friendlyGoalCenter() + Vector(0.75, 0);


    if (distanceSquared(field.friendlyGoalpostNeg(), ball.position()) > 0 &&
        distanceSquared(field.friendlyGoalpostPos(), ball.position()) > 0 && angle_to_cover != Angle::zero())
    {

        // compute block cone position
        Point goalie_pos = calculateBlockCone(
                positive_sweep.getEnd(), negative_sweep.getEnd(), ball.position(),
                robot_constants.robot_radius_m);

        //restrain goalie in semi circle
        double speed_factor = std::max(0.0, 1 - (std::abs(ball.velocity().y()) / BALL_MAX_SPEED_METERS_PER_SECOND)) ;
        Circle semi_circle = Circle(field.friendlyGoalCenter(), field.friendlyDefenseArea().yMax() * speed_factor);
        if (!contains(semi_circle, goalie_pos) || goalie_pos.x() < field.friendlyGoalCenter().x()){
            //project on to semi circle
            if(goalie_pos.x() < field.friendlyGoalCenter().x()){
                clamped_goalie_pos = field.friendlyGoalCenter() + ((ball.position() - field.friendlyGoalCenter()) * -1 ).normalize();
            } else {
                clamped_goalie_pos = field.friendlyGoalCenter() + ((ball.position() - field.friendlyGoalCenter()).normalize(semi_circle.radius()));
            }
        } else{
            clamped_goalie_pos = goalie_pos;
        }
    }


    return clamped_goalie_pos;

}

std::vector<Point> GoalieFSM::getIntersectionsBetweenBallVelocityAndFullGoalSegment(
    const Ball &ball, const Field &field)
{
    // compute intersection points from ball position and velocity
    Ray ball_ray = Ray(ball.position(), ball.velocity());

    // Create a segment along the goal line, slightly shortened to account for the
    // robot radius so as we move along the segment we don't try to run into the goal
    // posts. This will be used in case 3 as a fallback when we don't have an
    // intersection with the crease lines
    Segment full_goal_segment =
        Segment(field.friendlyGoalpostNeg() + Vector(0, -ROBOT_MAX_RADIUS_METERS),
                field.friendlyGoalpostPos() + Vector(0, ROBOT_MAX_RADIUS_METERS));

    return intersection(ball_ray, full_goal_segment);
}

Rectangle GoalieFSM::getNoChipRectangle(const Field &field)
{
    return Rectangle(
        field.friendlyGoalpostNeg(),
        field.friendlyGoalpostPos() + Vector(2 * ROBOT_MAX_RADIUS_METERS, 0));
}

std::optional<Point> GoalieFSM::restrainGoalieInRectangle(
    const Field &field, Point goalie_desired_position, Rectangle goalie_restricted_area)
{
    //           NW    pos_side   NE
    //            +---------------+
    //            |               |
    //            |               |
    //            |               |
    //       +----+               |
    //       |    |               |
    //       |    |               |
    // goal  |    |               | width
    //       |    |               |
    //       |    |               |
    //       |    |               |
    //       +----+               |
    //            |               |
    //            |               |
    //            |               |
    //           ++---------------+
    //           SW    neg_side   SE
    //
    // Given the goalies desired position and the restricted area,
    // first find the 3 intersections with each side of the restricted area
    // (width, pos_side, neg_side) and the line from the desired position to the
    // center of the friendly goal
    auto width_x_goal =
        intersection(Line(goalie_desired_position, field.friendlyGoalCenter()),
                     Line(goalie_restricted_area.posXPosYCorner(),
                          goalie_restricted_area.posXNegYCorner()));
    auto pos_side_x_goal =
        intersection(Line(goalie_desired_position, field.friendlyGoalCenter()),
                     Line(goalie_restricted_area.posXPosYCorner(),
                          goalie_restricted_area.negXPosYCorner()));
    auto neg_side_x_goal =
        intersection(Line(goalie_desired_position, field.friendlyGoalCenter()),
                     Line(goalie_restricted_area.posXNegYCorner(),
                          goalie_restricted_area.negXNegYCorner()));

    // if the goalie restricted area already contains the point, then we are
    // safe to move there.
    if (contains(goalie_restricted_area, goalie_desired_position))
    {
        return std::make_optional<Point>(goalie_desired_position);
    }
    // Due to the nature of the line intersection, its important to make sure the
    // corners are included, if the goalies desired position intersects with width
    // (see above), use those positions The last comparison is for the edge case when
    // the ball is behind the net
    else if (width_x_goal &&
             width_x_goal->y() <= goalie_restricted_area.posXPosYCorner().y() &&
             width_x_goal->y() >= goalie_restricted_area.posXNegYCorner().y() &&
             field.friendlyGoalCenter().x() <= goalie_desired_position.x())
    {
        return std::make_optional<Point>(*width_x_goal);
    }

    // if either two sides of the goal are intercepted, then use those positions
    else if (pos_side_x_goal &&
             pos_side_x_goal->x() <= goalie_restricted_area.posXPosYCorner().x() &&
             pos_side_x_goal->x() >= goalie_restricted_area.negXPosYCorner().x())
    {
        return std::make_optional<Point>(*pos_side_x_goal);
    }
    else if (neg_side_x_goal &&
             neg_side_x_goal->x() <= goalie_restricted_area.posXNegYCorner().x() &&
             neg_side_x_goal->x() >= goalie_restricted_area.negXNegYCorner().x())
    {
        return std::make_optional<Point>(*neg_side_x_goal);
    }

    // if there are no intersections (ex. ball behind net), then we are out of luck
    else
    {
        return std::nullopt;
    }
}

bool GoalieFSM::shouldPanic(const Update &event)
{
    double ball_speed_panic = goalie_tactic_config.ball_speed_panic();
    std::vector<Point> intersections =
        getIntersectionsBetweenBallVelocityAndFullGoalSegment(event.common.world.ball(),
                                                              event.common.world.field());
    return event.common.world.ball().velocity().length() > ball_speed_panic &&
           !intersections.empty();
}

bool GoalieFSM::shouldPivotChip(const Update &event)
{
    double ball_speed_panic = goalie_tactic_config.ball_speed_panic();
    return event.common.world.ball().velocity().length() <= ball_speed_panic &&
           event.common.world.field().pointInFriendlyDefenseArea(
               event.common.world.ball().position());
}

bool GoalieFSM::panicDone(const Update &event)
{
    double ball_speed_panic = goalie_tactic_config.ball_speed_panic();
    std::vector<Point> intersections =
        getIntersectionsBetweenBallVelocityAndFullGoalSegment(event.common.world.ball(),
                                                              event.common.world.field());

    return event.common.world.ball().velocity().length() <= ball_speed_panic ||
           intersections.empty();
}

void GoalieFSM::panic(const Update &event)
{
    std::vector<Point> intersections =
        getIntersectionsBetweenBallVelocityAndFullGoalSegment(event.common.world.ball(),
                                                              event.common.world.field());
    Point stop_ball_point = intersections[0];
    Point goalie_pos =
        closestPoint(event.common.robot.position(),
                     Segment(event.common.world.ball().position(), stop_ball_point));
    Angle goalie_orientation =
        (event.common.world.ball().position() - goalie_pos).orientation();

    Ball ball = event.common.world.ball();
    Robot robot = event.common.robot;

    event.common.set_primitive(createMovePrimitive(
            CREATE_MOTION_CONTROL(goalie_pos), goalie_orientation, 0.0,
            TbotsProto::DribblerMode::OFF, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, MAX_CHIP_DISTANCE},
            max_allowed_speed_mode, 0.0, event.common.robot.robotConstants(), std::optional<double>(), true));
}

void GoalieFSM::updatePivotKick(
    const Update &event, boost::sml::back::process<PivotKickFSM::Update> processEvent)
{
    double clear_origin_x =
        getNoChipRectangle(event.common.world.field()).xMax() + ROBOT_MAX_RADIUS_METERS;
    Point clear_origin = Point(clear_origin_x, event.common.world.ball().position().y());

    Point goalie_pos = event.common.robot.position();

    double clear_area_width =
        1.8;  // intersect between circle with radius of max chip distance and
              // straight line 1 meter away perpendicular to chip direction

    Point clear_corner_1 =
        Point(goalie_pos.x() + MAX_CHIP_DISTANCE - 1, clear_area_width / 2);
    Point clear_corner_2 =
        Point(goalie_pos.x() + MAX_CHIP_DISTANCE, -clear_area_width / 2);
    auto clear_area = std::make_optional<Rectangle>(clear_corner_1, clear_corner_2);

    std::vector<Circle> chip_targets =
        findGoodChipTargets(event.common.world, clear_area);

    Point clear_target;

    if (chip_targets.empty())
    {
        clear_target = event.common.world.field().centerPoint();
    }
    else
    {
        clear_target = chip_targets[0].origin();
    }

    Angle clear_direction =
        (clear_target - event.common.world.ball().position()).orientation();

    PivotKickFSM::ControlParams control_params{
        .kick_origin    = clear_origin,
        .kick_direction = clear_direction,
        .auto_chip_or_kick =
            AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, MAX_CHIP_DISTANCE},
    };

    // update the pivotkick fsm
    processEvent(PivotKickFSM::Update(control_params, event.common));
}

void GoalieFSM::positionToBlock(const Update &event)
{
    Point goalie_pos = getGoaliePositionToBlock(event.common.robot.robotConstants(), event.common.world.friendlyTeam(),
        event.common.world.ball(), event.common.world.field());
    Angle goalie_orientation =
        (event.common.world.ball().position() - goalie_pos).orientation();

    // what should the final goalie speed be, so that the goalie accelerates
    // faster
    auto goalie_final_speed = goalie_tactic_config.goalie_final_speed();

    event.common.set_primitive(createMovePrimitive(
            CREATE_MOTION_CONTROL(goalie_pos), goalie_orientation, goalie_final_speed,
            TbotsProto::DribblerMode::OFF, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, MAX_CHIP_DISTANCE},
            max_allowed_speed_mode, 0.0, event.common.robot.robotConstants(), std::optional<double>(), true));
}

bool GoalieFSM::ballInDefenseArea(const Update &event)
{
    return contains(event.common.world.field().friendlyDefenseArea(),
                    event.common.world.ball().position());
}

bool GoalieFSM::shouldMoveToGoalLine(const Update &event)
{
    return event.control_params.should_move_to_goal_line;
}

void GoalieFSM::moveToGoalLine(const Update &event)
{
    event.common.set_primitive(createMovePrimitive(
            CREATE_MOTION_CONTROL(event.common.world.field().friendlyGoalCenter()),
            Angle::zero(), 10, TbotsProto::DribblerMode::OFF,
            TbotsProto::BallCollisionType::AVOID,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0.0}, max_allowed_speed_mode, 0.0,
            event.common.robot.robotConstants(), std::optional<double>(), true));
}

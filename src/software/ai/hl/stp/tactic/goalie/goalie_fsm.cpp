#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"

Point GoalieFSM::getGoaliePositionToBlock(const Team &friendly_team,
    const Ball &ball, const Field &field,
    TbotsProto::GoalieTacticConfig goalie_tactic_config)
{
    // compute angle between two vectors, negative goal post to ball and positive
    // goal post to ball

    Segment s1 = Segment(ball.position(), field.friendlyGoalpostPos());
    Segment s2 = Segment(ball.position(), field.friendlyGoalpostNeg());

    static constexpr double INTERSECTION_INCREMENT_INTERVAL = 0.1;

    for (auto robot : friendly_team.getAllRobots()){
        if (robot.id() == friendly_team.getGoalieId()){
            continue;
        }
        Circle robot_circle = Circle(robot.position(), robot.robotConstants().robot_radius_m);

        while(intersects(robot_circle, s1) || intersects(robot_circle, s2) || s2.getEnd().y() > s1.getEnd().y()){
            if (intersects(robot_circle, s1)){
                Point new_end = Point(s1.getEnd().x(), s1.getEnd().y()-INTERSECTION_INCREMENT_INTERVAL);
                s1.setEnd(new_end);
            }

            if (intersects(robot_circle, s2)){
                Point new_end = Point(s2.getEnd().x(), s2.getEnd().y()+INTERSECTION_INCREMENT_INTERVAL);
                s2.setEnd(new_end);
            }

        }
    }

    std::cout<<s2.getEnd()<<" , "<<s1.getEnd()<<std::endl;

    Angle block_cone_angle = acuteAngle(s2.getEnd(), ball.position(),
                                        s1.getEnd());


    std::optional<Point> clamped_goalie_pos = std::nullopt;

    if (distanceSquared(field.friendlyGoalpostNeg(), ball.position()) > 0 &&
        distanceSquared(field.friendlyGoalpostPos(), ball.position()) > 0 &&
        block_cone_angle != Angle::zero())
    {
        // how far in should the goalie wedge itself into the block cone, to block
        // balls
        auto block_cone_radius = goalie_tactic_config.block_cone_radius();

        // compute block cone position, allowing 1 ROBOT_MAX_RADIUS_METERS extra on
        // either side
        Point goalie_pos = calculateBlockCone(
            s1.getEnd(), s2.getEnd(), ball.position(),
            block_cone_radius * block_cone_angle.toRadians());

        // restrain the goalie in the defense area, if the goalie cannot be
        // restrained or if there is no proper intersection, then we safely default to
        // center of the goal
        clamped_goalie_pos =
            restrainGoalieInRectangle(field, goalie_pos, field.friendlyDefenseArea());
        std::cout<<*clamped_goalie_pos<<std::endl;
    }

    // if the goalie could not be restrained in the defense area,
    // then the ball must be either on a really sharp angle to the net where
    // its impossible to get a shot, or the ball is behind the net, in which
    // case we snap to either post
    if (!clamped_goalie_pos)
    {
        if (ball.position().y() > 0)
        {
            return field.friendlyGoalpostPos() + Vector(0, -ROBOT_MAX_RADIUS_METERS);
        }
        else
        {
            return field.friendlyGoalpostNeg() + Vector(0, ROBOT_MAX_RADIUS_METERS);
        }
    }
    else
    {
        std::cout<<"returning goalie position"<<std::endl;
        return *clamped_goalie_pos;
    }
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


    //check if we are going to make it on time
    std::optional<Duration> ball_time_to_position =
            ball.getTimeToMoveDistance(distance(goalie_pos, ball.position()));


    Duration robot_time_to_pos = robot.getTimeToPosition(goalie_pos);

    if(robot_time_to_pos > ball_time_to_position){
        std::cout<<"WERE NOT GONNA MAKE IT"<<std::endl;

        //find the final speed at which we can make it

    }

    std::cout<<"robot speed = "<<event.common.robot.velocity().length()<<std::endl;
    event.common.set_primitive(createMovePrimitive(
        CREATE_MOTION_CONTROL(goalie_pos), goalie_orientation, 6.0,
        TbotsProto::DribblerMode::OFF, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, YEET_CHIP_DISTANCE_METERS},
        max_allowed_speed_mode, 0.0, event.common.robot.robotConstants()));
}

void GoalieFSM::updatePivotKick(
    const Update &event, boost::sml::back::process<PivotKickFSM::Update> processEvent)
{
    double clear_origin_x =
        getNoChipRectangle(event.common.world.field()).xMax() + ROBOT_MAX_RADIUS_METERS;
    Point clear_origin = Point(clear_origin_x, event.common.world.ball().position().y());

    Angle clear_direction = (event.common.world.ball().position() -
                             event.common.world.field().friendlyGoalCenter())
                                .orientation();

    PivotKickFSM::ControlParams control_params{
        .kick_origin    = clear_origin,
        .kick_direction = clear_direction,
        .auto_chip_or_kick =
            AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, YEET_CHIP_DISTANCE_METERS},
    };

    // update the pivotkick fsm
    processEvent(PivotKickFSM::Update(control_params, event.common));
}

void GoalieFSM::positionToBlock(const Update &event)
{
    Point goalie_pos = getGoaliePositionToBlock(event.common.world.friendlyTeam(),
        event.common.world.ball(), event.common.world.field(), goalie_tactic_config);
    Angle goalie_orientation =
        (event.common.world.ball().position() - goalie_pos).orientation();

    // what should the final goalie speed be, so that the goalie accelerates
    // faster
    auto goalie_final_speed = goalie_tactic_config.goalie_final_speed();

    event.common.set_primitive(createMovePrimitive(
        CREATE_MOTION_CONTROL(goalie_pos), goalie_orientation, goalie_final_speed,
        TbotsProto::DribblerMode::OFF, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, YEET_CHIP_DISTANCE_METERS},
        max_allowed_speed_mode, 0.0, event.common.robot.robotConstants()));
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
        event.common.robot.robotConstants()));
}

#include "backend_input/util/ros_messages.h"

#include "shared/constants.h"

thunderbots_msgs::Field MessageUtil::createFieldMsgFromFieldGeometry(
    const SSL_GeometryFieldSize &field_data)
{
    // We can't guarantee the order that any geometry elements are passed to us in, so
    // We map the name of each line/arc to the actual object so we can refer to them
    // consistantly
    std::map<std::string, SSL_FieldCicularArc> ssl_circular_arcs;
    std::map<std::string, SSL_FieldLineSegment> ssl_field_lines;

    // Circular arcs
    //
    // Arc names:
    // CenterCircle
    for (int i = 0; i < field_data.field_arcs_size(); i++)
    {
        const SSL_FieldCicularArc &arc = field_data.field_arcs(i);
        std::string arc_name           = arc.name();
        ssl_circular_arcs[arc_name]    = arc;
    }

    // Field Lines
    //
    // Line names:
    // TopTouchLine
    // BottomTouchLine
    // LeftGoalLine
    // RightGoalLine
    // HalfwayLine
    // CenterLine
    // LeftPenaltyStretch
    // RightPenaltyStretch
    // RightGoalTopLine
    // RightGoalBottomLine
    // RightGoalDepthLine
    // LeftGoalTopLine
    // LeftGoalBottomLine
    // LeftGoalDepthLine
    // LeftFieldLeftPenaltyStretch
    // LeftFieldRightPenaltyStretch
    // RightFieldLeftPenaltyStretch
    // RightFieldRightPenaltyStretch
    for (int i = 0; i < field_data.field_lines_size(); i++)
    {
        const SSL_FieldLineSegment &line = field_data.field_lines(i);
        std::string line_name            = line.name();
        ssl_field_lines[line_name]       = line;
    }

    thunderbots_msgs::Field field_msg;

    // Make sure we convert ALL fields into meters before returning
    field_msg.field_length = field_data.field_length() * METERS_PER_MILLIMETER;
    field_msg.field_width  = field_data.field_width() * METERS_PER_MILLIMETER;
    Point defense_length_p1 =
        Point(ssl_field_lines["LeftFieldLeftPenaltyStretch"].p1().x(),
              ssl_field_lines["LeftFieldLeftPenaltyStretch"].p1().y());
    Point defense_length_p2 =
        Point(ssl_field_lines["LeftFieldLeftPenaltyStretch"].p2().x(),
              ssl_field_lines["LeftFieldLeftPenaltyStretch"].p2().y());
    field_msg.defense_length =
        (defense_length_p2 - defense_length_p1).len() * METERS_PER_MILLIMETER;
    Point defense_width_p1 = Point(ssl_field_lines["LeftPenaltyStretch"].p1().x(),
                                   ssl_field_lines["LeftPenaltyStretch"].p1().y());
    Point defense_width_p2 = Point(ssl_field_lines["LeftPenaltyStretch"].p2().x(),
                                   ssl_field_lines["LeftPenaltyStretch"].p2().y());
    field_msg.defense_width =
        (defense_width_p1 - defense_width_p2).len() * METERS_PER_MILLIMETER;
    field_msg.goal_width     = field_data.goalwidth() * METERS_PER_MILLIMETER;
    field_msg.boundary_width = field_data.boundary_width() * METERS_PER_MILLIMETER;
    field_msg.center_circle_radius =
        ssl_circular_arcs["CenterCircle"].radius() * METERS_PER_MILLIMETER;

    return field_msg;
}

thunderbots_msgs::Ball MessageUtil::createBallMsgFromFilteredBallData(
    const FilteredBallData &filtered_ball_data)
{
    thunderbots_msgs::Ball ball_msg;

    ball_msg.position.x = filtered_ball_data.position.x();
    ball_msg.position.y = filtered_ball_data.position.y();

    ball_msg.velocity.x = filtered_ball_data.velocity.x();
    ball_msg.velocity.y = filtered_ball_data.velocity.y();

    ball_msg.timestamp_microseconds =
        Timestamp::getMicroseconds(filtered_ball_data.timestamp);

    return ball_msg;
}

thunderbots_msgs::Robot MessageUtil::createRobotMsgFromFilteredRobotData(
    const FilteredRobotData &filtered_robot_data)
{
    thunderbots_msgs::Robot robot_msg;

    robot_msg.id = filtered_robot_data.id;

    robot_msg.position.x = filtered_robot_data.position.x();
    robot_msg.position.y = filtered_robot_data.position.y();

    robot_msg.velocity.x = filtered_robot_data.velocity.x();
    robot_msg.velocity.y = filtered_robot_data.velocity.y();

    robot_msg.orientation = filtered_robot_data.orientation.toRadians();

    robot_msg.timestamp_nanoseconds_since_epoch = static_cast<unsigned long>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::microseconds(
                Timestamp::getMicroseconds(filtered_robot_data.timestamp)))
            .count());

    return robot_msg;
}

thunderbots_msgs::Team MessageUtil::createTeamMsgFromFilteredRobotData(
    const std::vector<FilteredRobotData> &filtered_team_data)
{
    thunderbots_msgs::Team team_msg;

    for (const FilteredRobotData &filtered_robot_data : filtered_team_data)
    {
        thunderbots_msgs::Robot robot_msg =
            createRobotMsgFromFilteredRobotData(filtered_robot_data);
        team_msg.robots.emplace_back(robot_msg);
    }

    return team_msg;
}

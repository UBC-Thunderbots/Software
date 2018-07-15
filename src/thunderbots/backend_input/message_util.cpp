#include "message_util.h"

// TODO: Create unit conversion functions
#define MILLIMETERS_TO_METERS_FLOAT 1000.0

thunderbots_msgs::Field VisionUtil::createFieldMsg(
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
    field_msg.field_length  = field_data.field_length() / MILLIMETERS_TO_METERS_FLOAT;
    field_msg.field_width   = field_data.field_width() / MILLIMETERS_TO_METERS_FLOAT;
    Point defense_length_p1 = Point(
        ssl_field_lines["LeftFieldLeftPenaltyStretch"].p1().x(),
        ssl_field_lines["LeftFieldLeftPenaltyStretch"].p1().y());
    Point defense_length_p2 = Point(
        ssl_field_lines["LeftFieldLeftPenaltyStretch"].p2().x(),
        ssl_field_lines["LeftFieldLeftPenaltyStretch"].p2().y());
    field_msg.defense_length =
        (defense_length_p2 - defense_length_p1).len() / MILLIMETERS_TO_METERS_FLOAT;
    Point defense_width_p1 = Point(
        ssl_field_lines["LeftPenaltyStretch"].p1().x(),
        ssl_field_lines["LeftPenaltyStretch"].p1().y());
    Point defense_width_p2 = Point(
        ssl_field_lines["LeftPenaltyStretch"].p2().x(),
        ssl_field_lines["LeftPenaltyStretch"].p2().y());
    field_msg.defense_width =
        (defense_width_p1 - defense_width_p2).len() / MILLIMETERS_TO_METERS_FLOAT;
    field_msg.goal_width     = field_data.goalwidth() / MILLIMETERS_TO_METERS_FLOAT;
    field_msg.boundary_width = field_data.boundary_width() / MILLIMETERS_TO_METERS_FLOAT;
    field_msg.center_circle_radius =
        ssl_circular_arcs["CenterCircle"].radius() / MILLIMETERS_TO_METERS_FLOAT;

    return field_msg;
}

thunderbots_msgs::Ball VisionUtil::createBallMsg(
    const Point &position, const Point &velocity)
{
    thunderbots_msgs::Ball ball_msg;

    ball_msg.position.x = position.x();
    ball_msg.position.y = position.y();

    ball_msg.velocity.x = velocity.x();
    ball_msg.velocity.y = velocity.y();

    return ball_msg;
}

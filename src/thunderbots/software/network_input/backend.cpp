#include "network_input/backend.h"

#include "proto/messages_robocup_ssl_detection.pb.h"
#include "proto/messages_robocup_ssl_geometry.pb.h"
#include "shared/constants.h"
#include "util/constants.h"

// We can initialize the field_state with all zeroes here because this state will never
// be accessed by an external observer to this class. the getFieldData must be called to
// get any field data which will update the state with the given protobuf data
Backend::Backend()
    : field_state(0, 0, 0, 0, 0, 0, 0),
      ball_state(Point(), Vector(), Timestamp::fromSeconds(0)),
      friendly_team_state(Duration::fromMilliseconds(
          Util::Constants::ROBOT_DEBOUNCE_DURATION_MILLISECONDS)),
      enemy_team_state(Duration::fromMilliseconds(
          Util::Constants::ROBOT_DEBOUNCE_DURATION_MILLISECONDS)),
      ball_filter(),
      friendly_team_filter(),
      enemy_team_filter()
{
}

Field Backend::getFieldData(const SSL_GeometryData &geometry_packet)
{
    if (geometry_packet.has_field())
    {
        SSL_GeometryFieldSize field_data = geometry_packet.field();
        Field field                      = createFieldFromPacketGeometry(field_data);

        field_state = field;
    }

    return field_state;
}

Field Backend::createFieldFromPacketGeometry(
    const SSL_GeometryFieldSize &packet_geometry) const
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
    for (int i = 0; i < packet_geometry.field_arcs_size(); i++)
    {
        const SSL_FieldCicularArc &arc = packet_geometry.field_arcs(i);
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
    for (int i = 0; i < packet_geometry.field_lines_size(); i++)
    {
        const SSL_FieldLineSegment &line = packet_geometry.field_lines(i);
        std::string line_name            = line.name();
        ssl_field_lines[line_name]       = line;
    }

    thunderbots_msgs::Field field_msg;

    // Extract the data we care about and convert all units to meters
    double field_length   = packet_geometry.field_length() * METERS_PER_MILLIMETER;
    double field_width    = packet_geometry.field_width() * METERS_PER_MILLIMETER;
    double goal_width     = packet_geometry.goalwidth() * METERS_PER_MILLIMETER;
    double boundary_width = packet_geometry.boundary_width() * METERS_PER_MILLIMETER;
    double center_circle_radius =
        ssl_circular_arcs["CenterCircle"].radius() * METERS_PER_MILLIMETER;

    // We arbitraily use the left side here since the left and right sides are identical
    Point defense_length_p1 =
        Point(ssl_field_lines["LeftFieldLeftPenaltyStretch"].p1().x(),
              ssl_field_lines["LeftFieldLeftPenaltyStretch"].p1().y());
    Point defense_length_p2 =
        Point(ssl_field_lines["LeftFieldLeftPenaltyStretch"].p2().x(),
              ssl_field_lines["LeftFieldLeftPenaltyStretch"].p2().y());
    double defense_length =
        (defense_length_p2 - defense_length_p1).len() * METERS_PER_MILLIMETER;

    // We arbitraily use the left side here since the left and right sides are identical
    Point defense_width_p1 = Point(ssl_field_lines["LeftPenaltyStretch"].p1().x(),
                                   ssl_field_lines["LeftPenaltyStretch"].p1().y());
    Point defense_width_p2 = Point(ssl_field_lines["LeftPenaltyStretch"].p2().x(),
                                   ssl_field_lines["LeftPenaltyStretch"].p2().y());
    double defense_width =
        (defense_width_p1 - defense_width_p2).len() * METERS_PER_MILLIMETER;

    Field field = Field(field_length, field_width, defense_length, defense_width,
                        goal_width, boundary_width, center_circle_radius);
    return field;
}

Ball Backend::getFilteredBallData(const std::vector<SSL_DetectionFrame> &detections)
{
    auto ball_detections = std::vector<SSLBallDetection>();

    for (const auto &detection : detections)
    {
        for (const SSL_DetectionBall &ball : detection.balls())
        {
            // Convert all data to meters and radians
            SSLBallDetection ball_detection;
            ball_detection.position =
                Point(ball.x() * METERS_PER_MILLIMETER, ball.y() * METERS_PER_MILLIMETER);
            ball_detection.confidence = ball.confidence();
            ball_detection.timestamp  = Timestamp::fromSeconds(detection.t_capture());
            ball_detections.push_back(ball_detection);
        }
    }

    Ball updated_ball_state = ball_filter.getFilteredData(ball_state, ball_detections);
    ball_state              = updated_ball_state;

    return ball_state;
}

Team Backend::getFilteredFriendlyTeamData(std::vector<SSL_DetectionFrame> detections)
{
    auto friendly_robot_detections = std::vector<SSLRobotDetection>();

    // Collect all the visible robots from all camera frames
    for (const auto &detection : detections)
    {
        auto ssl_robots = detection.robots_yellow();
        if (Util::Constants::FRIENDLY_TEAM_COLOUR == BLUE)
        {
            ssl_robots = detection.robots_blue();
        }

        for (const auto &friendly_robot_detection : ssl_robots)
        {
            SSLRobotDetection robot_detection;

            robot_detection.id = friendly_robot_detection.robot_id();
            robot_detection.position =
                Point(friendly_robot_detection.x() * METERS_PER_MILLIMETER,
                      friendly_robot_detection.y() * METERS_PER_MILLIMETER);
            robot_detection.orientation =
                Angle::ofRadians(friendly_robot_detection.orientation());
            robot_detection.confidence = friendly_robot_detection.confidence();
            robot_detection.timestamp  = Timestamp::fromSeconds(detection.t_capture());

            friendly_robot_detections.push_back(robot_detection);
        }
    }

    Team updated_team_state = friendly_team_filter.getFilteredData(
        friendly_team_state, friendly_robot_detections);
    friendly_team_state = updated_team_state;

    return friendly_team_state;
}

Team Backend::getFilteredEnemyTeamData(const std::vector<SSL_DetectionFrame> &detections)
{
    auto enemy_robot_detections = std::vector<SSLRobotDetection>();

    // Collect all the visible robots from all camera frames
    for (const auto &detection : detections)
    {
        auto ssl_robots = detection.robots_blue();
        if (Util::Constants::FRIENDLY_TEAM_COLOUR == BLUE)
        {
            ssl_robots = detection.robots_yellow();
        }

        for (const auto &enemy_robot_detection : ssl_robots)
        {
            SSLRobotDetection robot_detection;

            robot_detection.id = enemy_robot_detection.robot_id();
            robot_detection.position =
                Point(enemy_robot_detection.x() * METERS_PER_MILLIMETER,
                      enemy_robot_detection.y() * METERS_PER_MILLIMETER);
            robot_detection.orientation =
                Angle::ofRadians(enemy_robot_detection.orientation());
            robot_detection.confidence = enemy_robot_detection.confidence();
            robot_detection.timestamp  = Timestamp::fromSeconds(detection.t_capture());

            enemy_robot_detections.push_back(robot_detection);
        }
    }

    Team updated_team_state =
        enemy_team_filter.getFilteredData(enemy_team_state, enemy_robot_detections);
    enemy_team_state = updated_team_state;

    return enemy_team_state;
}

std::optional<thunderbots_msgs::RefboxData> Backend::getRefboxDataMsg(
    const Referee &packet)
{
    thunderbots_msgs::RefboxData refbox_data;
    refbox_data.command.command = getTeamCommand(packet.command());
    setOurFieldSide(packet.blue_team_on_positive_half());
    auto designated_position = refboxGlobalToLocalPoint(packet.designated_position());
    refbox_data.ball_placement_point.x = designated_position.x();
    refbox_data.ball_placement_point.y = designated_position.y();
    refbox_data.packet_timestamp       = packet.packet_timestamp();
    refbox_data.command_timestamp      = packet.command_timestamp();

    thunderbots_msgs::RefboxTeamInfo blue   = getTeamInfo(packet.blue());
    thunderbots_msgs::RefboxTeamInfo yellow = getTeamInfo(packet.yellow());

    if (Util::Constants::FRIENDLY_TEAM_COLOUR == TeamColour::BLUE)
    {
        refbox_data.us   = blue;
        refbox_data.them = yellow;
    }
    else
    {
        refbox_data.us   = yellow;
        refbox_data.them = blue;
    }

    return std::make_optional<thunderbots_msgs::RefboxData>(refbox_data);
}

// this maps a protobuf Referee_Command enum to its ROS message equivalent
// this map is used when we are on the blue team
const static std::unordered_map<Referee::Command, int> blue_team_command_map = {
    {Referee_Command_HALT, thunderbots_msgs::RefboxCommand::HALT},
    {Referee_Command_STOP, thunderbots_msgs::RefboxCommand::STOP},
    {Referee_Command_NORMAL_START, thunderbots_msgs::RefboxCommand::NORMAL_START},
    {Referee_Command_FORCE_START, thunderbots_msgs::RefboxCommand::FORCE_START},
    {Referee_Command_PREPARE_KICKOFF_BLUE,
     thunderbots_msgs::RefboxCommand::PREPARE_KICKOFF_US},
    {Referee_Command_PREPARE_KICKOFF_YELLOW,
     thunderbots_msgs::RefboxCommand::PREPARE_KICKOFF_THEM},
    {Referee_Command_PREPARE_PENALTY_BLUE,
     thunderbots_msgs::RefboxCommand::PREPARE_PENALTY_US},
    {Referee_Command_PREPARE_PENALTY_YELLOW,
     thunderbots_msgs::RefboxCommand::PREPARE_PENALTY_THEM},
    {Referee_Command_DIRECT_FREE_BLUE, thunderbots_msgs::RefboxCommand::DIRECT_FREE_US},
    {Referee_Command_DIRECT_FREE_YELLOW,
     thunderbots_msgs::RefboxCommand::DIRECT_FREE_THEM},
    {Referee_Command_INDIRECT_FREE_BLUE,
     thunderbots_msgs::RefboxCommand::INDIRECT_FREE_US},
    {Referee_Command_INDIRECT_FREE_YELLOW,
     thunderbots_msgs::RefboxCommand::INDIRECT_FREE_THEM},
    {Referee_Command_TIMEOUT_BLUE, thunderbots_msgs::RefboxCommand::TIMEOUT_US},
    {Referee_Command_TIMEOUT_YELLOW, thunderbots_msgs::RefboxCommand::TIMEOUT_THEM},
    {Referee_Command_GOAL_BLUE, thunderbots_msgs::RefboxCommand::GOAL_US},
    {Referee_Command_GOAL_YELLOW, thunderbots_msgs::RefboxCommand::GOAL_THEM},
    {Referee_Command_BALL_PLACEMENT_BLUE,
     thunderbots_msgs::RefboxCommand::BALL_PLACEMENT_US},
    {Referee_Command_BALL_PLACEMENT_YELLOW,
     thunderbots_msgs::RefboxCommand::BALL_PLACEMENT_THEM}};

// this maps a protobuf Referee_Command enum to its ROS message equivalent
// this map is used when we are on the yellow team
const static std::unordered_map<Referee::Command, int> yellow_team_command_map = {
    {Referee_Command_HALT, thunderbots_msgs::RefboxCommand::HALT},
    {Referee_Command_STOP, thunderbots_msgs::RefboxCommand::STOP},
    {Referee_Command_NORMAL_START, thunderbots_msgs::RefboxCommand::NORMAL_START},
    {Referee_Command_FORCE_START, thunderbots_msgs::RefboxCommand::FORCE_START},
    {Referee_Command_PREPARE_KICKOFF_BLUE,
     thunderbots_msgs::RefboxCommand::PREPARE_KICKOFF_THEM},
    {Referee_Command_PREPARE_KICKOFF_YELLOW,
     thunderbots_msgs::RefboxCommand::PREPARE_KICKOFF_US},
    {Referee_Command_PREPARE_PENALTY_BLUE,
     thunderbots_msgs::RefboxCommand::PREPARE_PENALTY_THEM},
    {Referee_Command_PREPARE_PENALTY_YELLOW,
     thunderbots_msgs::RefboxCommand::PREPARE_PENALTY_US},
    {Referee_Command_DIRECT_FREE_BLUE, thunderbots_msgs::RefboxCommand::DIRECT_FREE_THEM},
    {Referee_Command_DIRECT_FREE_YELLOW, thunderbots_msgs::RefboxCommand::DIRECT_FREE_US},
    {Referee_Command_INDIRECT_FREE_BLUE,
     thunderbots_msgs::RefboxCommand::INDIRECT_FREE_THEM},
    {Referee_Command_INDIRECT_FREE_YELLOW,
     thunderbots_msgs::RefboxCommand::INDIRECT_FREE_US},
    {Referee_Command_TIMEOUT_BLUE, thunderbots_msgs::RefboxCommand::TIMEOUT_THEM},
    {Referee_Command_TIMEOUT_YELLOW, thunderbots_msgs::RefboxCommand::TIMEOUT_US},
    {Referee_Command_GOAL_BLUE, thunderbots_msgs::RefboxCommand::GOAL_THEM},
    {Referee_Command_GOAL_YELLOW, thunderbots_msgs::RefboxCommand::GOAL_US},
    {Referee_Command_BALL_PLACEMENT_BLUE,
     thunderbots_msgs::RefboxCommand::BALL_PLACEMENT_THEM},
    {Referee_Command_BALL_PLACEMENT_YELLOW,
     thunderbots_msgs::RefboxCommand::BALL_PLACEMENT_US}};

int32_t Backend::getTeamCommand(const Referee::Command &command)
{
    auto our_team_colour = Util::Constants::FRIENDLY_TEAM_COLOUR;
    if (our_team_colour == TeamColour::BLUE)
    {
        return blue_team_command_map.at(command);
    }
    else
    {
        return yellow_team_command_map.at(command);
    }
}

Point Backend::refboxGlobalToLocalPoint(const Referee::Point &point)
{
    if (our_field_side == FieldSide::WEST)
    {
        return Point(-point.x(), -point.y());
    }
    else
    {
        return Point(point.x(), point.y());
    }
}

void Backend::setOurFieldSide(bool blue_team_on_positive_half)
{
    if (blue_team_on_positive_half)
    {
        if (Util::Constants::FRIENDLY_TEAM_COLOUR == TeamColour::BLUE)
        {
            our_field_side = FieldSide::WEST;
        }
        else
        {
            our_field_side = FieldSide::EAST;
        }
    }
    else
    {
        if (Util::Constants::FRIENDLY_TEAM_COLOUR == TeamColour::BLUE)
        {
            our_field_side = FieldSide::EAST;
        }
        else
        {
            our_field_side = FieldSide::WEST;
        }
    }
}

thunderbots_msgs::RefboxTeamInfo Backend::getTeamInfo(const Referee::TeamInfo &team_info)
{
    thunderbots_msgs::RefboxTeamInfo refbox_team_info;
    refbox_team_info.team_name = team_info.name();
    refbox_team_info.score     = team_info.score();
    refbox_team_info.red_cards = team_info.red_cards();
    for (auto card_time : team_info.yellow_card_times())
    {
        refbox_team_info.yellow_card_times.push_back(card_time);
    }
    refbox_team_info.timeouts     = team_info.timeouts();
    refbox_team_info.timeout_time = team_info.timeout_time();
    refbox_team_info.goalie       = team_info.goalkeeper();
    return refbox_team_info;
}

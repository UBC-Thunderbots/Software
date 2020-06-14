#include "software/backend/input/network/networking/network_filter.h"

#include "shared/constants.h"
#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/proto/messages_robocup_ssl_detection.pb.h"
#include "software/proto/messages_robocup_ssl_geometry.pb.h"
#include "software/sensor_fusion/refbox_data.h"

// We can initialize the field_state with all zeroes here because this state will never
// be accessed by an external observer to this class. the getFieldData must be called to
// get any field data which will update the state with the given protobuf data
NetworkFilter::NetworkFilter(std::shared_ptr<const RefboxConfig> refbox_config)
    : field_state(),
      ball_state(Point(), Vector(), Timestamp::fromSeconds(0)),
      friendly_team_state(Duration::fromMilliseconds(
          Util::Constants::ROBOT_DEBOUNCE_DURATION_MILLISECONDS)),
      enemy_team_state(Duration::fromMilliseconds(
          Util::Constants::ROBOT_DEBOUNCE_DURATION_MILLISECONDS)),
      ball_filter(BallFilter::DEFAULT_MIN_BUFFER_SIZE,
                  BallFilter::DEFAULT_MAX_BUFFER_SIZE),
      friendly_team_filter(),
      enemy_team_filter(),
      refbox_config(refbox_config)
{
}

std::optional<Field> NetworkFilter::getFieldData(const SSL_GeometryData &geometry_packet)
{
    if (geometry_packet.has_field())
    {
        SSL_GeometryFieldSize field_data = geometry_packet.field();
        Field field                      = createFieldFromPacketGeometry(field_data);

        field_state = field;
    }

    return field_state;
}

Field NetworkFilter::createFieldFromPacketGeometry(
    const SSL_GeometryFieldSize &packet_geometry) const
{
    // We can't guarantee the order that any geometry elements are passed to us in, so
    // We map the name of each line/arc to the actual object so we can refer to them
    // consistently
    std::map<std::string, SSL_FieldCircularArc> ssl_circular_arcs;
    std::map<std::string, SSL_FieldLineSegment> ssl_field_lines;

    // Circular arcs
    //
    // Arc names:
    // CenterCircle
    for (int i = 0; i < packet_geometry.field_arcs_size(); i++)
    {
        const SSL_FieldCircularArc &arc = packet_geometry.field_arcs(i);
        std::string arc_name            = arc.name();
        ssl_circular_arcs[arc_name]     = arc;
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

    // Extract the data we care about and convert all units to meters
    double field_length   = packet_geometry.field_length() * METERS_PER_MILLIMETER;
    double field_width    = packet_geometry.field_width() * METERS_PER_MILLIMETER;
    double goal_width     = packet_geometry.goal_width() * METERS_PER_MILLIMETER;
    double goal_depth     = packet_geometry.goal_depth() * METERS_PER_MILLIMETER;
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
        (defense_length_p2 - defense_length_p1).length() * METERS_PER_MILLIMETER;

    // We arbitraily use the left side here since the left and right sides are identical
    Point defense_width_p1 = Point(ssl_field_lines["LeftPenaltyStretch"].p1().x(),
                                   ssl_field_lines["LeftPenaltyStretch"].p1().y());
    Point defense_width_p2 = Point(ssl_field_lines["LeftPenaltyStretch"].p2().x(),
                                   ssl_field_lines["LeftPenaltyStretch"].p2().y());
    double defense_width =
        (defense_width_p1 - defense_width_p2).length() * METERS_PER_MILLIMETER;

    Field field = Field(field_length, field_width, defense_length, defense_width,
                        goal_depth, goal_width, boundary_width, center_circle_radius);
    return field;
}

std::optional<TimestampedBallState> NetworkFilter::getFilteredBallData(
    const std::vector<SSL_DetectionFrame> &detections)
{
    auto ball_detections = std::vector<BallDetection>();

    for (const auto &detection : detections)
    {
        for (const SSL_DetectionBall &ball : detection.balls())
        {
            // Convert all data to meters and radians
            BallDetection ball_detection;
            ball_detection.position =
                Point(ball.x() * METERS_PER_MILLIMETER, ball.y() * METERS_PER_MILLIMETER);
            ball_detection.timestamp = Timestamp::fromSeconds(detection.t_capture());

            bool ball_position_invalid =
                refbox_config->MinValidX()->value() > ball_detection.position.x() ||
                refbox_config->MaxValidX()->value() < ball_detection.position.x();
            bool ignore_ball = refbox_config->IgnoreInvalidCameraData()->value() &&
                               ball_position_invalid;
            if (!ignore_ball)
            {
                ball_detections.push_back(ball_detection);
            }
        }
    }

    if (field_state)
    {
        std::optional<Ball> new_ball =
            ball_filter.getFilteredData(ball_detections, *field_state);
        if (new_ball)
        {
            ball_state = new_ball->currentState();
        }
    }

    return ball_state;
}

Team NetworkFilter::getFilteredFriendlyTeamData(
    const std::vector<SSL_DetectionFrame> &detections)
{
    auto friendly_robot_detections = std::vector<RobotDetection>();

    // Collect all the visible robots from all camera frames
    for (const auto &detection : detections)
    {
        auto ssl_robots = detection.robots_yellow();
        if (!refbox_config->FriendlyColorYellow()->value())
        {
            ssl_robots = detection.robots_blue();
        }

        for (const auto &friendly_robot_detection : ssl_robots)
        {
            RobotDetection robot_detection;

            robot_detection.id = friendly_robot_detection.robot_id();
            robot_detection.position =
                Point(friendly_robot_detection.x() * METERS_PER_MILLIMETER,
                      friendly_robot_detection.y() * METERS_PER_MILLIMETER);
            robot_detection.orientation =
                Angle::fromRadians(friendly_robot_detection.orientation());
            robot_detection.confidence = friendly_robot_detection.confidence();
            robot_detection.timestamp  = Timestamp::fromSeconds(detection.t_capture());


            bool robot_position_invalid =
                refbox_config->MinValidX()->value() > robot_detection.position.x() ||
                refbox_config->MaxValidX()->value() < robot_detection.position.x();
            bool ignore_robot = refbox_config->IgnoreInvalidCameraData()->value() &&
                                robot_position_invalid;
            if (!ignore_robot)
            {
                friendly_robot_detections.push_back(robot_detection);
            }
        }
    }

    Team updated_team_state = friendly_team_filter.getFilteredData(
        friendly_team_state, friendly_robot_detections);
    friendly_team_state = updated_team_state;

    return friendly_team_state;
}

Team NetworkFilter::getFilteredEnemyTeamData(
    const std::vector<SSL_DetectionFrame> &detections)
{
    auto enemy_robot_detections = std::vector<RobotDetection>();

    // Collect all the visible robots from all camera frames
    for (const auto &detection : detections)
    {
        auto ssl_robots = detection.robots_blue();
        if (!refbox_config->FriendlyColorYellow()->value())
        {
            ssl_robots = detection.robots_yellow();
        }

        for (const auto &enemy_robot_detection : ssl_robots)
        {
            RobotDetection robot_detection;

            robot_detection.id = enemy_robot_detection.robot_id();
            robot_detection.position =
                Point(enemy_robot_detection.x() * METERS_PER_MILLIMETER,
                      enemy_robot_detection.y() * METERS_PER_MILLIMETER);
            robot_detection.orientation =
                Angle::fromRadians(enemy_robot_detection.orientation());
            robot_detection.confidence = enemy_robot_detection.confidence();
            robot_detection.timestamp  = Timestamp::fromSeconds(detection.t_capture());

            bool robot_position_invalid =
                refbox_config->MinValidX()->value() > robot_detection.position.x() ||
                refbox_config->MaxValidX()->value() < robot_detection.position.x();
            bool ignore_robot = refbox_config->IgnoreInvalidCameraData()->value() &&
                                robot_position_invalid;
            if (!ignore_robot)
            {
                enemy_robot_detections.push_back(robot_detection);
            }
        }
    }

    Team updated_team_state =
        enemy_team_filter.getFilteredData(enemy_team_state, enemy_robot_detections);
    enemy_team_state = updated_team_state;

    return enemy_team_state;
}

RefboxGameState NetworkFilter::getRefboxGameState(const Referee &packet)
{
    return getTeamCommand(packet.command());
}

// this maps a protobuf Referee_Command enum to its equivalent internal type
// this map is used when we are on the blue team
const static std::unordered_map<Referee::Command, RefboxGameState> blue_team_command_map =
    {{Referee_Command_HALT, RefboxGameState::HALT},
     {Referee_Command_STOP, RefboxGameState::STOP},
     {Referee_Command_NORMAL_START, RefboxGameState::NORMAL_START},
     {Referee_Command_FORCE_START, RefboxGameState::FORCE_START},
     {Referee_Command_PREPARE_KICKOFF_BLUE, RefboxGameState::PREPARE_KICKOFF_US},
     {Referee_Command_PREPARE_KICKOFF_YELLOW, RefboxGameState::PREPARE_KICKOFF_THEM},
     {Referee_Command_PREPARE_PENALTY_BLUE, RefboxGameState::PREPARE_PENALTY_US},
     {Referee_Command_PREPARE_PENALTY_YELLOW, RefboxGameState::PREPARE_PENALTY_THEM},
     {Referee_Command_DIRECT_FREE_BLUE, RefboxGameState::DIRECT_FREE_US},
     {Referee_Command_DIRECT_FREE_YELLOW, RefboxGameState::DIRECT_FREE_THEM},
     {Referee_Command_INDIRECT_FREE_BLUE, RefboxGameState::INDIRECT_FREE_US},
     {Referee_Command_INDIRECT_FREE_YELLOW, RefboxGameState::INDIRECT_FREE_THEM},
     {Referee_Command_TIMEOUT_BLUE, RefboxGameState::TIMEOUT_US},
     {Referee_Command_TIMEOUT_YELLOW, RefboxGameState::TIMEOUT_THEM},
     {Referee_Command_GOAL_BLUE, RefboxGameState::GOAL_US},
     {Referee_Command_GOAL_YELLOW, RefboxGameState::GOAL_THEM},
     {Referee_Command_BALL_PLACEMENT_BLUE, RefboxGameState::BALL_PLACEMENT_US},
     {Referee_Command_BALL_PLACEMENT_YELLOW, RefboxGameState::BALL_PLACEMENT_THEM}};

// this maps a protobuf Referee_Command enum to its equivalent internal type
// this map is used when we are on the yellow team
const static std::unordered_map<Referee::Command, RefboxGameState>
    yellow_team_command_map = {
        {Referee_Command_HALT, RefboxGameState::HALT},
        {Referee_Command_STOP, RefboxGameState::STOP},
        {Referee_Command_NORMAL_START, RefboxGameState::NORMAL_START},
        {Referee_Command_FORCE_START, RefboxGameState::FORCE_START},
        {Referee_Command_PREPARE_KICKOFF_BLUE, RefboxGameState::PREPARE_KICKOFF_THEM},
        {Referee_Command_PREPARE_KICKOFF_YELLOW, RefboxGameState::PREPARE_KICKOFF_US},
        {Referee_Command_PREPARE_PENALTY_BLUE, RefboxGameState::PREPARE_PENALTY_THEM},
        {Referee_Command_PREPARE_PENALTY_YELLOW, RefboxGameState::PREPARE_PENALTY_US},
        {Referee_Command_DIRECT_FREE_BLUE, RefboxGameState::DIRECT_FREE_THEM},
        {Referee_Command_DIRECT_FREE_YELLOW, RefboxGameState::DIRECT_FREE_US},
        {Referee_Command_INDIRECT_FREE_BLUE, RefboxGameState::INDIRECT_FREE_THEM},
        {Referee_Command_INDIRECT_FREE_YELLOW, RefboxGameState::INDIRECT_FREE_US},
        {Referee_Command_TIMEOUT_BLUE, RefboxGameState::TIMEOUT_THEM},
        {Referee_Command_TIMEOUT_YELLOW, RefboxGameState::TIMEOUT_US},
        {Referee_Command_GOAL_BLUE, RefboxGameState::GOAL_THEM},
        {Referee_Command_GOAL_YELLOW, RefboxGameState::GOAL_US},
        {Referee_Command_BALL_PLACEMENT_BLUE, RefboxGameState::BALL_PLACEMENT_THEM},
        {Referee_Command_BALL_PLACEMENT_YELLOW, RefboxGameState::BALL_PLACEMENT_US}};

RefboxGameState NetworkFilter::getTeamCommand(const Referee::Command &command)
{
    if (!refbox_config->FriendlyColorYellow()->value())
    {
        return blue_team_command_map.at(command);
    }
    else
    {
        return yellow_team_command_map.at(command);
    }
}

void NetworkFilter::setOurFieldSide(bool blue_team_on_positive_half)
{
    if (blue_team_on_positive_half)
    {
        if (!refbox_config->FriendlyColorYellow()->value())
        {
            our_field_side = FieldSide::NEG_X;
        }
        else
        {
            our_field_side = FieldSide::POS_X;
        }
    }
    else
    {
        if (!refbox_config->FriendlyColorYellow()->value())
        {
            our_field_side = FieldSide::POS_X;
        }
        else
        {
            our_field_side = FieldSide::NEG_X;
        }
    }
}

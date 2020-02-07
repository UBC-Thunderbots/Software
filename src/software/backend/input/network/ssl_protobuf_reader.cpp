#include "software/backend/input/network/ssl_protobuf_reader.h"

// We can initialize the field_state with all zeroes here because this state will never
// be accessed by an external observer to this class. the getFieldData must be called to
// get any field data which will update the state with the given protobuf data
SSLProtobufReader::SSLProtobufReader()
    : field_state(0, 0, 0, 0, 0, 0, 0, Timestamp::fromSeconds(0))
{
}

Field SSLProtobufReader::getFieldData(const SSL_GeometryData &geometry_packet)
{
    SSL_GeometryFieldSize field_data = geometry_packet.field();
    std::optional<Field> field_opt   = createFieldFromPacketGeometry(field_data);

    if (field_opt)
    {
        field_state = *field_opt;
    }
    else
    {
        LOG(WARNING)
            << "Invalid field packet has been detected, which means field_state may be unreliable "
            << "and the createFieldFromPacketGeometry may be parsing using the wrong proto format";
    }

    return field_state;
}

std::optional<Field> SSLProtobufReader::createFieldFromPacketGeometry(
    const SSL_GeometryFieldSize &packet_geometry) const
{
    // We can't guarantee the order that any geometry elements are passed to us in, so
    // We map the name of each line/arc to the actual object so we can refer to them
    // consistently
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

    // Check that CenterCircle exists before using it
    auto ssl_center_circle = ssl_circular_arcs.find("CenterCircle");
    if (ssl_center_circle == ssl_circular_arcs.end())
    {
        return std::nullopt;
    }

    // Extract the data we care about and convert all units to meters
    double field_length   = packet_geometry.field_length() * METERS_PER_MILLIMETER;
    double field_width    = packet_geometry.field_width() * METERS_PER_MILLIMETER;
    double goal_width     = packet_geometry.goalwidth() * METERS_PER_MILLIMETER;
    double boundary_width = packet_geometry.boundary_width() * METERS_PER_MILLIMETER;
    double center_circle_radius =
        ssl_center_circle->second.radius() * METERS_PER_MILLIMETER;

    // Check that LeftFieldLeftPenaltyStretch exists before using it
    auto ssl_left_field_left_penalty_stretch =
        ssl_field_lines.find("LeftFieldLeftPenaltyStretch");
    if (ssl_left_field_left_penalty_stretch == ssl_field_lines.end())
    {
        return std::nullopt;
    }

    // We arbitraily use the left side here since the left and right sides are identical
    Point defense_length_p1 = Point(ssl_left_field_left_penalty_stretch->second.p1().x(),
                                    ssl_left_field_left_penalty_stretch->second.p1().y());
    Point defense_length_p2 = Point(ssl_left_field_left_penalty_stretch->second.p2().x(),
                                    ssl_left_field_left_penalty_stretch->second.p2().y());
    double defense_length =
        (defense_length_p2 - defense_length_p1).length() * METERS_PER_MILLIMETER;

    // Check that LeftPenaltyStretch exists before using it
    auto ssl_left_penalty_stretch = ssl_field_lines.find("LeftPenaltyStretch");
    if (ssl_left_penalty_stretch == ssl_field_lines.end())
    {
        return std::nullopt;
    }

    // We arbitraily use the left side here since the left and right sides are identical
    Point defense_width_p1 = Point(ssl_left_penalty_stretch->second.p1().x(),
                                   ssl_left_penalty_stretch->second.p1().y());
    Point defense_width_p2 = Point(ssl_left_penalty_stretch->second.p2().x(),
                                   ssl_left_penalty_stretch->second.p2().y());
    double defense_width =
        (defense_width_p1 - defense_width_p2).length() * METERS_PER_MILLIMETER;

    Field field =
        Field(field_length, field_width, defense_length, defense_width, goal_width,
              boundary_width, center_circle_radius, Timestamp::fromSeconds(0));
    return field;
}

std::vector<BallDetection> SSLProtobufReader::getBallDetections(
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

            // TODO remove Util::DynamicParameters as part of
            // https://github.com/UBC-Thunderbots/Software/issues/960
            bool ball_position_invalid =
                Util::DynamicParameters->getAIControlConfig()
                        ->getRefboxConfig()
                        ->MinValidX()
                        ->value() > ball_detection.position.x() ||
                Util::DynamicParameters->getAIControlConfig()
                        ->getRefboxConfig()
                        ->MaxValidX()
                        ->value() < ball_detection.position.x();
            bool ignore_ball = Util::DynamicParameters->getAIControlConfig()
                                   ->getRefboxConfig()
                                   ->IgnoreInvalidCameraData()
                                   ->value() &&
                               ball_position_invalid;
            if (!ignore_ball)
            {
                ball_detections.push_back(ball_detection);
            }
        }
    }

    return ball_detections;
}

std::vector<RobotDetection> SSLProtobufReader::getTeamDetections(
    const std::vector<SSL_DetectionFrame> &detections, TeamType team_type)
{
    std::vector<RobotDetection> robot_detections = std::vector<RobotDetection>();

    // Collect all the visible robots from all camera frames
    for (const auto &detection : detections)
    {
        auto ssl_robots = detection.robots_blue();
        // TODO remove Util::DynamicParameters as part of
        // https://github.com/UBC-Thunderbots/Software/issues/960
        if ((team_type == TeamType::FRIENDLY &&
             Util::DynamicParameters->getAIControlConfig()
                 ->getRefboxConfig()
                 ->FriendlyColorYellow()
                 ->value()) ||
            (team_type == TeamType::ENEMY &&
             !Util::DynamicParameters->getAIControlConfig()
                  ->getRefboxConfig()
                  ->FriendlyColorYellow()
                  ->value()))
        {
            ssl_robots = detection.robots_yellow();
        }

        for (const auto &ssl_robot_detection : ssl_robots)
        {
            RobotDetection robot_detection;

            robot_detection.id = ssl_robot_detection.robot_id();
            robot_detection.position =
                Point(ssl_robot_detection.x() * METERS_PER_MILLIMETER,
                      ssl_robot_detection.y() * METERS_PER_MILLIMETER);
            robot_detection.orientation =
                Angle::fromRadians(ssl_robot_detection.orientation());
            robot_detection.confidence = ssl_robot_detection.confidence();
            robot_detection.timestamp  = Timestamp::fromSeconds(detection.t_capture());


            bool robot_position_invalid =
                Util::DynamicParameters->getAIControlConfig()
                        ->getRefboxConfig()
                        ->MinValidX()
                        ->value() > robot_detection.position.x() ||
                Util::DynamicParameters->getAIControlConfig()
                        ->getRefboxConfig()
                        ->MaxValidX()
                        ->value() < robot_detection.position.x();
            bool ignore_robot = Util::DynamicParameters->getAIControlConfig()
                                    ->getRefboxConfig()
                                    ->IgnoreInvalidCameraData()
                                    ->value() &&
                                robot_position_invalid;
            if (!ignore_robot)
            {
                robot_detections.push_back(robot_detection);
            }
        }
    }

    return robot_detections;
}

RefboxData SSLProtobufReader::getRefboxData(const Referee &packet)
{
    // SSL Referee proto messages' `Command` fields map to `RefboxGameState` data
    // structures
    RefboxGameState game_state      = getRefboxGameState(packet.command());
    RefboxGameState next_game_state = getRefboxGameState(packet.next_command());

    TeamInfo friendly_team_info, enemy_team_info;

    TeamInfo yellow_team_info(
        packet.yellow().name(), packet.yellow().score(), packet.yellow().red_cards(),
        std::vector<int>(packet.yellow().yellow_card_times().begin(),
                         packet.yellow().yellow_card_times().begin()),
        packet.yellow().yellow_cards(), packet.yellow().timeouts(),
        packet.yellow().timeout_time(), packet.yellow().goalkeeper(),
        packet.yellow().foul_counter(), packet.yellow().ball_placement_failures(),
        packet.yellow().can_place_ball(), packet.yellow().max_allowed_bots());

    TeamInfo blue_team_info(
        packet.blue().name(), packet.blue().score(), packet.blue().red_cards(),
        std::vector<int>(packet.blue().yellow_card_times().begin(),
                         packet.blue().yellow_card_times().begin()),
        packet.blue().yellow_cards(), packet.blue().timeouts(),
        packet.blue().timeout_time(), packet.blue().goalkeeper(),
        packet.blue().foul_counter(), packet.blue().ball_placement_failures(),
        packet.blue().can_place_ball(), packet.blue().max_allowed_bots());

    if (Util::DynamicParameters->getAIControlConfig()
            ->getRefboxConfig()
            ->FriendlyColorYellow()
            ->value())
    {
        friendly_team_info = yellow_team_info;
        enemy_team_info    = blue_team_info;
    }
    else
    {
        friendly_team_info = blue_team_info;
        enemy_team_info    = yellow_team_info;
    }

    RefboxStage stage = getRefboxStage(packet.stage());

    return RefboxData(
        Timestamp::fromMilliseconds(packet.packet_timestamp() / 1000),
        Timestamp::fromMilliseconds(packet.command_timestamp() / 1000),
        packet.command_counter(),
        Point(packet.designated_position().x(), packet.designated_position().y()),
        packet.blue_team_on_positive_half(),
        Duration::fromMilliseconds(packet.current_action_time_remaining() / 1000),
        friendly_team_info, enemy_team_info, game_state, next_game_state, stage,
        std::vector<GameEvent>(packet.game_events().begin(), packet.game_events().end()),
        std::vector<ProposedGameEvent>(packet.proposed_game_events().begin(),
                                       packet.proposed_game_events().end()));
}

// this maps a protobuf Referee_Command enum to its ROS message equivalent
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

// this maps a protobuf Referee_Command enum to its ROS message equivalent
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

RefboxGameState SSLProtobufReader::getRefboxGameState(const Referee::Command &command)
{
    if (!Util::DynamicParameters->getAIControlConfig()
             ->getRefboxConfig()
             ->FriendlyColorYellow()
             ->value())
    {
        return blue_team_command_map.at(command);
    }
    else
    {
        return yellow_team_command_map.at(command);
    }
}

// this maps a protobuf Referee_Stage enum to its RefboxStage equivalent
const static std::unordered_map<Referee::Stage, RefboxStage> refbox_stage_map = {
    {Referee_Stage_NORMAL_FIRST_HALF_PRE, RefboxStage::NORMAL_FIRST_HALF_PRE},
    {Referee_Stage_NORMAL_FIRST_HALF, RefboxStage::NORMAL_FIRST_HALF},
    {Referee_Stage_NORMAL_HALF_TIME, RefboxStage::NORMAL_HALF_TIME},
    {Referee_Stage_NORMAL_SECOND_HALF_PRE, RefboxStage::NORMAL_SECOND_HALF_PRE},
    {Referee_Stage_NORMAL_SECOND_HALF, RefboxStage::NORMAL_SECOND_HALF},
    {Referee_Stage_EXTRA_TIME_BREAK, RefboxStage::EXTRA_TIME_BREAK},
    {Referee_Stage_EXTRA_FIRST_HALF_PRE, RefboxStage::EXTRA_FIRST_HALF_PRE},
    {Referee_Stage_EXTRA_FIRST_HALF, RefboxStage::EXTRA_FIRST_HALF},
    {Referee_Stage_EXTRA_HALF_TIME, RefboxStage::EXTRA_HALF_TIME},
    {Referee_Stage_EXTRA_SECOND_HALF_PRE, RefboxStage::EXTRA_SECOND_HALF_PRE},
    {Referee_Stage_EXTRA_SECOND_HALF, RefboxStage::EXTRA_SECOND_HALF},
    {Referee_Stage_PENALTY_SHOOTOUT_BREAK, RefboxStage::PENALTY_SHOOTOUT_BREAK},
    {Referee_Stage_PENALTY_SHOOTOUT, RefboxStage::PENALTY_SHOOTOUT},
    {Referee_Stage_POST_GAME, RefboxStage::POST_GAME}};

RefboxStage SSLProtobufReader::getRefboxStage(const Referee::Stage &stage)
{
    return refbox_stage_map.at(stage);
}

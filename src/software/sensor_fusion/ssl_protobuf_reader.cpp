#include "software/sensor_fusion/ssl_protobuf_reader.h"

#include "software/proto/message_translation/ssl_geometry_message_translator.h"

SSLProtobufReader::SSLProtobufReader() {}

std::optional<Field> SSLProtobufReader::getField(
    const SSL_GeometryData &geometry_packet) const
{
    SSL_GeometryFieldSize field_data = geometry_packet.field();

    auto ssl_center_circle =
        findCircularArc(field_data.field_arcs(), SSLCircularArcs::CENTER_CIRCLE);
    if (!ssl_center_circle)
    {
        return std::nullopt;
    }

    // Extract the data we care about and convert all units to meters
    double field_length         = field_data.field_length() * METERS_PER_MILLIMETER;
    double field_width          = field_data.field_width() * METERS_PER_MILLIMETER;
    double goal_width           = field_data.goal_width() * METERS_PER_MILLIMETER;
    double goal_depth           = field_data.goal_depth() * METERS_PER_MILLIMETER;
    double boundary_width       = field_data.boundary_width() * METERS_PER_MILLIMETER;
    double center_circle_radius = ssl_center_circle->radius() * METERS_PER_MILLIMETER;

    auto ssl_left_field_left_penalty_stretch = findLineSegment(
        field_data.field_lines(), SSLFieldLines::LEFT_FIELD_LEFT_PENALTY_STRETCH);
    if (!ssl_left_field_left_penalty_stretch)
    {
        return std::nullopt;
    }

    // We arbitraily use the left side here since the left and right sides are identical
    Point defense_length_p1 = Point(ssl_left_field_left_penalty_stretch->p1().x(),
                                    ssl_left_field_left_penalty_stretch->p1().y());
    Point defense_length_p2 = Point(ssl_left_field_left_penalty_stretch->p2().x(),
                                    ssl_left_field_left_penalty_stretch->p2().y());
    double defense_length =
        (defense_length_p2 - defense_length_p1).length() * METERS_PER_MILLIMETER;

    auto ssl_left_penalty_stretch =
        findLineSegment(field_data.field_lines(), SSLFieldLines::LEFT_PENALTY_STRETCH);
    if (!ssl_left_penalty_stretch)
    {
        return std::nullopt;
    }

    // We arbitraily use the left side here since the left and right sides are identical
    Point defense_width_p1 =
        Point(ssl_left_penalty_stretch->p1().x(), ssl_left_penalty_stretch->p1().y());
    Point defense_width_p2 =
        Point(ssl_left_penalty_stretch->p2().x(), ssl_left_penalty_stretch->p2().y());
    double defense_width =
        (defense_width_p1 - defense_width_p2).length() * METERS_PER_MILLIMETER;

    Field field = Field(field_length, field_width, defense_length, defense_width,
                        goal_depth, goal_width, boundary_width, center_circle_radius);
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

VisionDetection SSLProtobufReader::getVisionDetection(
    const SSL_DetectionFrame &detection_frame)
{
    std::vector<BallDetection> ball_detections;
    std::vector<RobotDetection> friendly_team_detections;
    std::vector<RobotDetection> enemy_team_detections;
    Timestamp latest_timestamp;
    SSL_DetectionFrame detection = detection_frame;

    // We invert the field side if we explicitly choose to override the values
    // provided by refbox. The 'defending_positive_side' parameter dictates the side
    // we are defending if we are overriding the value
    // TODO remove as part of https://github.com/UBC-Thunderbots/Software/issues/960
    if (Util::DynamicParameters->getAIControlConfig()
            ->getRefboxConfig()
            ->OverrideRefboxDefendingSide()
            ->value() &&
        Util::DynamicParameters->getAIControlConfig()
            ->getRefboxConfig()
            ->DefendingPositiveSide()
            ->value())
    {
        invertFieldSide(detection);
    }

    if (isCameraEnabled(detection))
    {
        // filter protos into internal data structures
        ball_detections          = getBallDetections({detection});
        friendly_team_detections = getTeamDetections({detection}, TeamType::FRIENDLY);
        enemy_team_detections    = getTeamDetections({detection}, TeamType::ENEMY);
    }

    latest_timestamp = Timestamp::fromSeconds(detection.t_capture());

    return VisionDetection(ball_detections, friendly_team_detections,
                           enemy_team_detections, latest_timestamp);
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

RefboxGameState SSLProtobufReader::getRefboxGameState(const Referee &packet)
{
    if (!Util::DynamicParameters->getAIControlConfig()
             ->getRefboxConfig()
             ->FriendlyColorYellow()
             ->value())
    {
        return blue_team_command_map.at(packet.command());
    }
    else
    {
        return yellow_team_command_map.at(packet.command());
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

RefboxStage SSLProtobufReader::getRefboxStage(const Referee &packet)
{
    return refbox_stage_map.at(packet.stage());
}

void SSLProtobufReader::invertFieldSide(SSL_DetectionFrame &frame)
{
    for (SSL_DetectionBall &ball : *frame.mutable_balls())
    {
        ball.set_x(-ball.x());
        ball.set_y(-ball.y());
    }
    for (const auto &team : {frame.mutable_robots_yellow(), frame.mutable_robots_blue()})
    {
        for (SSL_DetectionRobot &robot : *team)
        {
            robot.set_x(-robot.x());
            robot.set_y(-robot.y());
            robot.set_orientation(robot.orientation() + M_PI);
        }
    }
}

bool SSLProtobufReader::isCameraEnabled(const SSL_DetectionFrame &detection)
{
    bool camera_disabled = false;
    switch (detection.camera_id())
    {
        // TODO: create an array of dynamic params to index into with camera_id()
        // may be resolved by https://github.com/UBC-Thunderbots/Software/issues/960
        case 0:
            camera_disabled =
                Util::DynamicParameters->getCameraConfig()->IgnoreCamera_0()->value();
            break;
        case 1:
            camera_disabled =
                Util::DynamicParameters->getCameraConfig()->IgnoreCamera_1()->value();
            break;
        case 2:
            camera_disabled =
                Util::DynamicParameters->getCameraConfig()->IgnoreCamera_2()->value();
            break;
        case 3:
            camera_disabled =
                Util::DynamicParameters->getCameraConfig()->IgnoreCamera_3()->value();
            break;
        default:
            LOG(WARNING) << "An unkown camera id was detected, disabled by default "
                         << "id: " << detection.camera_id() << std::endl;
            camera_disabled = true;
            break;
    }
    return !camera_disabled;
}

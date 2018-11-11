#include "backend.h"

#include "backend_input/util/ros_messages.h"
#include "proto/messages_robocup_ssl_detection.pb.h"
#include "proto/messages_robocup_ssl_geometry.pb.h"
#include "shared/constants.h"
#include "util/constants.h"

Backend::Backend() : ball_filter(), friendly_team_filter(), enemy_team_filter() {}

std::optional<thunderbots_msgs::Field> Backend::getFieldMsg(
    const SSL_WrapperPacket &packet)
{
    if (packet.has_geometry())
    {
        const SSL_GeometryData &geom       = packet.geometry();
        const SSL_GeometryFieldSize &field = geom.field();
        thunderbots_msgs::Field field_msg =
            MessageUtil::createFieldMsgFromFieldGeometry(field);
        return std::optional<thunderbots_msgs::Field>(field_msg);
    }

    return std::nullopt;
}

std::optional<thunderbots_msgs::Ball> Backend::getFilteredBallMsg(
    const SSL_WrapperPacket &packet, const AITimestamp &timestamp)
{
    if (packet.has_detection())
    {
        const SSL_DetectionFrame &detection = packet.detection();

        std::vector<SSLBallData> ball_detections = std::vector<SSLBallData>();
        for (const SSL_DetectionBall &ball : detection.balls())
        {
            SSLBallData ball_data;
            ball_data.position =
                Point(ball.x() * METERS_PER_MILLIMETER, ball.y() * METERS_PER_MILLIMETER);
            ball_data.confidence = ball.confidence();
            ball_data.timestamp  = timestamp;
            ball_detections.push_back(ball_data);
        }

        FilteredBallData filtered_ball_data =
            ball_filter.getFilteredData(ball_detections);
        thunderbots_msgs::Ball ball_msg =
            MessageUtil::createBallMsgFromFilteredBallData(filtered_ball_data);
        return ball_msg;
    }

    return std::nullopt;
}

std::optional<thunderbots_msgs::Team> Backend::getFilteredFriendlyTeamMsg(
    const SSL_WrapperPacket &packet, const AITimestamp &timestamp)
{
    if (packet.has_detection())
    {
        const SSL_DetectionFrame &detection = packet.detection();

        std::vector<SSLRobotData> friendly_team_robot_data = std::vector<SSLRobotData>();

        auto ssl_robots = detection.robots_yellow();
        if (Util::Constants::FRIENDLY_TEAM_COLOUR == BLUE)
        {
            ssl_robots = detection.robots_blue();
        }

        for (const SSL_DetectionRobot &friendly_robot : ssl_robots)
        {
            SSLRobotData new_robot_data;

            new_robot_data.id          = friendly_robot.robot_id();
            new_robot_data.position    = Point(friendly_robot.x(), friendly_robot.y());
            new_robot_data.orientation = Angle::ofRadians(friendly_robot.orientation());
            new_robot_data.confidence  = friendly_robot.confidence();
            new_robot_data.timestamp   = timestamp;

            friendly_team_robot_data.emplace_back(new_robot_data);
        }

        std::vector<FilteredRobotData> filtered_friendly_team_data =
            friendly_team_filter.getFilteredData(friendly_team_robot_data);

        thunderbots_msgs::Team friendly_team_msg =
            MessageUtil::createTeamMsgFromFilteredRobotData(filtered_friendly_team_data);

        return friendly_team_msg;
    }

    return std::nullopt;
}


std::optional<thunderbots_msgs::Team> Backend::getFilteredEnemyTeamMsg(
    const SSL_WrapperPacket &packet, const AITimestamp &timestamp)
{
    if (packet.has_detection())
    {
        const SSL_DetectionFrame &detection = packet.detection();

        std::vector<SSLRobotData> enemy_team_robot_data = std::vector<SSLRobotData>();

        auto ssl_robots = detection.robots_yellow();
        if (Util::Constants::FRIENDLY_TEAM_COLOUR == YELLOW)
        {
            ssl_robots = detection.robots_blue();
        }

        for (const SSL_DetectionRobot &enemy_robot : ssl_robots)
        {
            SSLRobotData new_robot_data;

            new_robot_data.id          = enemy_robot.robot_id();
            new_robot_data.position    = Point(enemy_robot.x(), enemy_robot.y());
            new_robot_data.orientation = Angle::ofRadians(enemy_robot.orientation());
            new_robot_data.confidence  = enemy_robot.confidence();

            enemy_team_robot_data.emplace_back(new_robot_data);
        }

        std::vector<FilteredRobotData> filtered_enemy_team_data =
            enemy_team_filter.getFilteredData(enemy_team_robot_data);

        thunderbots_msgs::Team enemy_team_msg =
            MessageUtil::createTeamMsgFromFilteredRobotData(filtered_enemy_team_data);

        return enemy_team_msg;
    }

    return std::nullopt;
}

std::optional<thunderbots_msgs::RefboxData> Backend::getRefboxDataMsg(const Referee &packet) {
    thunderbots_msgs::RefboxData refbox_data;
    thunderbots_msgs::RefboxCommand command;
    command.command = getTeamCommand(packet.command(), Util::Constants::FRIENDLY_TEAM_COLOUR);

    return std::optional<thunderbots_msgs::RefboxData>();
}

// my apologies for another monster switch statement
int32_t Backend::getTeamCommand(Referee::Command command, TeamColour our_team_colour) {
    switch(command) {
        case Referee_Command_HALT:
            return thunderbots_msgs::RefboxCommand::HALT;
        case Referee_Command_STOP:
            return thunderbots_msgs::RefboxCommand::STOP;
        case Referee_Command_NORMAL_START:
            return thunderbots_msgs::RefboxCommand::NORMAL_START;
        case Referee_Command_FORCE_START:
            return thunderbots_msgs::RefboxCommand::FORCE_START;
        case Referee_Command_PREPARE_KICKOFF_YELLOW:
            if(our_team_colour == TeamColour::YELLOW) {
                return thunderbots_msgs::RefboxCommand::PREPARE_KICKOFF_US;
            } else {
                return thunderbots_msgs::RefboxCommand::PREPARE_KICKOFF_THEM;
            }
        case Referee_Command_PREPARE_KICKOFF_BLUE:
            if(our_team_colour == TeamColour::YELLOW) {
                return thunderbots_msgs::RefboxCommand::PREPARE_KICKOFF_THEM;
            } else {
                return thunderbots_msgs::RefboxCommand::PREPARE_KICKOFF_US;
            }
        case Referee_Command_PREPARE_PENALTY_YELLOW:
            if(our_team_colour == TeamColour::YELLOW) {
                return thunderbots_msgs::RefboxCommand::PREPARE_PENALTY_US;
            } else {
                return thunderbots_msgs::RefboxCommand::PREPARE_PENALTY_THEM;
            }
        case Referee_Command_PREPARE_PENALTY_BLUE:
            if(our_team_colour == TeamColour::YELLOW) {
                return thunderbots_msgs::RefboxCommand::PREPARE_PENALTY_THEM;
            } else {
                return thunderbots_msgs::RefboxCommand::PREPARE_PENALTY_US;
            }
        case Referee_Command_DIRECT_FREE_YELLOW:
            if(our_team_colour == TeamColour::YELLOW) {
                return thunderbots_msgs::RefboxCommand::DIRECT_FREE_US;
            } else {
                return thunderbots_msgs::RefboxCommand::DIRECT_FREE_THEM;
            }
        case Referee_Command_DIRECT_FREE_BLUE:
            if(our_team_colour == TeamColour::YELLOW) {
                return thunderbots_msgs::RefboxCommand::DIRECT_FREE_THEM;
            } else {
                return thunderbots_msgs::RefboxCommand::DIRECT_FREE_US;
            }
        case Referee_Command_INDIRECT_FREE_YELLOW:
            if(our_team_colour == TeamColour::YELLOW) {
                return thunderbots_msgs::RefboxCommand::INDIRECT_FREE_US;
            } else {
                return thunderbots_msgs::RefboxCommand::INDIRECT_FREE_THEM;
            }
        case Referee_Command_INDIRECT_FREE_BLUE:
            if(our_team_colour == TeamColour::YELLOW) {
                return thunderbots_msgs::RefboxCommand::INDIRECT_FREE_THEM;
            } else {
                return thunderbots_msgs::RefboxCommand::INDIRECT_FREE_US;
            }
        case Referee_Command_TIMEOUT_YELLOW:
            if(our_team_colour == TeamColour::YELLOW) {
                return thunderbots_msgs::RefboxCommand::TIMEOUT_US;
            } else {
                return thunderbots_msgs::RefboxCommand::TIMEOUT_THEM;
            }
        case Referee_Command_TIMEOUT_BLUE:
            if(our_team_colour == TeamColour::YELLOW) {
                return thunderbots_msgs::RefboxCommand::TIMEOUT_THEM;
            } else {
                return thunderbots_msgs::RefboxCommand::TIMEOUT_US;
            }
        case Referee_Command_GOAL_YELLOW:
            if(our_team_colour == TeamColour::YELLOW) {
                return thunderbots_msgs::RefboxCommand::GOAL_US;
            } else {
                return thunderbots_msgs::RefboxCommand::GOAL_THEM;
            }
        case Referee_Command_GOAL_BLUE:
            if(our_team_colour == TeamColour::YELLOW) {
                return thunderbots_msgs::RefboxCommand::GOAL_THEM;
            } else {
                return thunderbots_msgs::RefboxCommand::GOAL_US;
            }
        case Referee_Command_BALL_PLACEMENT_YELLOW:
            if(our_team_colour == TeamColour::YELLOW) {
                return thunderbots_msgs::RefboxCommand::BALL_PLACEMENT_US;
            } else {
                return thunderbots_msgs::RefboxCommand::BALL_PLACEMENT_THEM;
            }
        case Referee_Command_BALL_PLACEMENT_BLUE:
            if(our_team_colour == TeamColour::YELLOW) {
                return thunderbots_msgs::RefboxCommand::BALL_PLACEMENT_THEM;
            } else {
                return thunderbots_msgs::RefboxCommand::PREPARE_KICKOFF_THEM;
            }
    }
}

Point Backend::getTeamLocalCoordinates(Referee::Point point) {
    if (Util::Constants::FRIENDLY_FIELD_SIDE == FieldSide::WEST) {
        return Point(point.x(), point.y());
    } else {
        return Point(- point.x(), - point.y());
    }
}

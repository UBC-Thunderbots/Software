#pragma once

#include <optional>

#include "network_input/filter/ball_filter.h"
#include "network_input/filter/robot_filter.h"
#include "network_input/filter/robot_team_filter.h"
#include "proto/messages_robocup_ssl_wrapper.pb.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/Robot.h"
#include "thunderbots_msgs/Team.h"
#include "util/timestamp.h"

class Backend
{
   public:
    /**
     * Creates a new Backend for data input and filtering
     */
    explicit Backend();

    /**
     * Given a new protobuf packet, updates the Ball filter and returns a Ball message
     * containing the most up to date filtered Ball data
     *
     * @param packet The SSL Vision packet containing new data
     * @param timestamp The timestamp at which the packet was received
     *
     * @return a Ball message containing the most up to date filtered Ball data. If the
     * packet does not contain any new Ball information, returns std::nullopt
     */
    std::optional<thunderbots_msgs::Ball> getFilteredBallMsg(
        const SSL_WrapperPacket &packet, const AITimestamp &timestamp);

    /**
     * Given a new protobuf packet, returns a Field message containing the most up to date
     * Field geometry
     *
     * @param packet The SSL Vision packet containing new data
     *
     * @return a Field message containing the most up to date Field geometry. If the
     * packet does not contain any new Field geometry, returns std::nullopt
     */
    std::optional<thunderbots_msgs::Field> getFieldMsg(const SSL_WrapperPacket &packet);

    /**
     * Given a new protobuf packet, updates the friendly team filter and returns a Team
     * message containing the most up to date filtered friendly team data
     *
     * @param packet The SSL Vision packet containing new data
     * @param timestamp The timestamp at which the packet was received
     *
     * @return a Team message containing the most up to date filtered friendly team data.
     * If the packet does not contain any new Team or Robot data, returns std::nullopt
     */
    std::optional<thunderbots_msgs::Team> getFilteredFriendlyTeamMsg(
        const SSL_WrapperPacket &packet, const AITimestamp &timestamp);

    /**
     * Given a new protobuf packet, updates the enemy team filter and returns a Team
     * message containing the most up to date filtered enemy team data
     *
     * @param packet The SSL Vision packet containing new data
     * @param timestamp The timestamp at which the packet was received
     *
     * @return a Team message containing the most up to date filtered enemy team data.
     * If the packet does not contain any new Team or Robot data, returns std::nullopt
     */
    std::optional<thunderbots_msgs::Team> getFilteredEnemyTeamMsg(
        const SSL_WrapperPacket &packet, const AITimestamp &timestamp);

    virtual ~Backend() = default;

   private:
    BallFilter ball_filter;
    RobotTeamFilter friendly_team_filter;
    RobotTeamFilter enemy_team_filter;
};

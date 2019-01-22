#pragma once

#include <optional>

#include "ai/world/field.h"
#include "network_input/filter/ball_filter.h"
#include "network_input/filter/robot_filter.h"
#include "network_input/filter/robot_team_filter.h"
#include "proto/messages_robocup_ssl_wrapper.pb.h"
#include "proto/ssl_referee.pb.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/RefboxData.h"
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
     *
     * @return a Ball message containing the most up to date filtered Ball data. If the
     * packet does not contain any new Ball information, returns std::nullopt
     */
    std::optional<thunderbots_msgs::Ball> getFilteredBallMsg(
        const SSL_WrapperPacket &packet);

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
     *
     * @return a Team message containing the most up to date filtered friendly team data.
     * If the packet does not contain any new Team or Robot data, returns std::nullopt
     */
    std::optional<thunderbots_msgs::Team> getFilteredFriendlyTeamMsg(
        const SSL_WrapperPacket &packet);

    /**
     * Given a new protobuf packet, updates the enemy team filter and returns a Team
     * message containing the most up to date filtered enemy team data
     *
     * @param packet The SSL Vision packet containing new data
     *
     * @return a Team message containing the most up to date filtered enemy team data.
     * If the packet does not contain any new Team or Robot data, returns std::nullopt
     */
    std::optional<thunderbots_msgs::Team> getFilteredEnemyTeamMsg(
        const SSL_WrapperPacket &packet);

    std::optional<thunderbots_msgs::RefboxData> getRefboxDataMsg(const Referee &packet);


    virtual ~Backend() = default;

   private:
    BallFilter ball_filter;
    RobotTeamFilter friendly_team_filter;
    RobotTeamFilter enemy_team_filter;

    // backend *should* be the only part of the system that is aware of Refbox/Vision
    // global coordinates. To AI, +x will always be enemy and -x will always be friendly.
    FieldSide our_field_side;
    // TODO: the rest of backend should be spitting out coordinates transformed as above
    // https://github.com/UBC-Thunderbots/Software/issues/163

    void setOurFieldSide(bool blue_team_on_positive_half);

    int32_t getTeamCommand(const Referee::Command &command);
    Point getTeamLocalCoordinates(const Referee::Point &point);
    thunderbots_msgs::RefboxTeamInfo getTeamInfo(const Referee::TeamInfo &team_info);
};

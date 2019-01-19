#pragma once

#include "geom/point.h"
#include "network_input/filter/ball_filter.h"
#include "network_input/filter/robot_filter.h"
#include "proto/messages_robocup_ssl_geometry.pb.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/Robot.h"
#include "thunderbots_msgs/Team.h"
#include "util/timestamp.h"

class MessageUtil
{
   public:
    /**
     * Creates a Field msg given raw SSL Field Geometry data
     *
     * @param field_data the SSL Geometry packet containing field geometry data
     *
     * @return a Field message containing the field geometry information
     */
    static thunderbots_msgs::Field createFieldMsgFromFieldGeometry(
        const SSL_GeometryFieldSize &field_data);

    /**
     * Creates a Ball msg given the filtered data for a ball
     *
     * @param filtered_ball_data the filtered data for the ball
     *
     * @return a Ball message containing the ball information
     */
    static thunderbots_msgs::Ball createBallMsgFromFilteredBallData(
        const FilteredBallData &filtered_ball_data);

    /**
     * Creates a Robot msg given the Filtered Data for a robot
     *
     * @param filtered_robot_data the Filtered Data for a robot
     *
     * @return a Robot message containing the robot's information
     */
    static thunderbots_msgs::Robot createRobotMsgFromFilteredRobotData(
        const FilteredRobotData &filtered_robot_data);

    /**
     * Creates a Team msg given team data
     *
     * @param filtered_team_data A vector of robot data representing a team
     *
     * @return a Team message containing the information for the team
     */
    static thunderbots_msgs::Team createTeamMsgFromFilteredRobotData(
        const std::vector<FilteredRobotData> &filtered_team_data);
};

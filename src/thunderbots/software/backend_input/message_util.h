#pragma once

#include <backend_input/filter/robot_team_filter.h>
#include "geom/point.h"
#include "proto/messages_robocup_ssl_geometry.pb.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/Robot.h"
#include "thunderbots_msgs/Team.h"

class VisionUtil
{
   public:
    /**
     * Creates a Field msg given raw SSL Field Geometry data
     *
     * @param field_data the SSL Geometry packet containing field geometry data
     *
     * @return a Field message containing the field geometry information
     */
    static thunderbots_msgs::Field createFieldMsg(
        const SSL_GeometryFieldSize &field_data);

    /**
     * Creates a Ball msg given the position and velocity of a ball
     *
     * @param position the position of the ball
     * @param velocity the velocity of the ball
     *
     * @return a Ball message containing the ball information
     */
    static thunderbots_msgs::Ball createBallMsg(
        const Point &position, const Point &velocity);

    /**
     * Creates a Robot msg given the Filtered Data for a robot
     *
     * @param robot_data the Filtered Data for a robot
     *
     * @return a Robot message containing the robot's information
     */
    static thunderbots_msgs::Robot createRobotMsg(const FilteredRobotData &robot_data);

    /**
     * Creates a Team msg given team data
     *
     * @param team_data A vector of robot data representing a team
     *
     * @return a Team message containing the information for the team
     */
    static thunderbots_msgs::Team createTeamMsg(
        const std::vector<FilteredRobotData> &team_data);
};

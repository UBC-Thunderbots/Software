#include <ros/ros.h>
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/Team.h"

// Callbacks
void fieldUpdateCallback(const thunderbots_msgs::Field::ConstPtr &msg)
{
    thunderbots_msgs::Field field_msg = *msg;
}

void ballUpdateCallback(const thunderbots_msgs::Ball::ConstPtr &msg)
{
    thunderbots_msgs::Ball ball_msg = *msg;
}

void friendlyTeamUpdateCallback(const thunderbots_msgs::Team::ConstPtr &msg)
{
    thunderbots_msgs::Team friendly_team_msg = *msg;
}

void enemyTeamUpdateCallback(const thunderbots_msgs::Team::ConstPtr &msg)
{
    thunderbots_msgs::Team enemy_team_msg = *msg;
}

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "ai_logic");
    ros::NodeHandle n;

    // Create subscribers
    ros::Subscriber field_sub = n.subscribe("backend/field", 1, fieldUpdateCallback);
    ros::Subscriber ball_sub  = n.subscribe("backend/ball", 1, ballUpdateCallback);
    ros::Subscriber friendly_team_sub =
        n.subscribe("backend/friendly_team", 1, friendlyTeamUpdateCallback);
    ros::Subscriber enemy_team_sub =
        n.subscribe("backend/enemy_team", 1, enemyTeamUpdateCallback);

    // Main loop
    while (ros::ok())
    {
        // Spin once to let all necessary callbacks run
        ros::spinOnce();
    }

    return 0;
}

#include <ros/ros.h>
#include "ai/hl/stp/stphl.h"
#include "ai/intent.h"
#include "ai/world/ball.h"
#include "ai/world/field.h"
#include "ai/world/robot.h"
#include "ai/world/team.h"
#include "ai/world/world.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/Team.h"

// Member variables we need to maintain state
World world;
STPHL high_level;

// Callbacks
void fieldUpdateCallback(const thunderbots_msgs::Field::ConstPtr &msg)
{
    thunderbots_msgs::Field field_msg = *msg;

    Field new_field = Field();
    new_field.updateDimensions(field_msg);

    world.updateField(new_field);
}

void ballUpdateCallback(const thunderbots_msgs::Ball::ConstPtr &msg)
{
    thunderbots_msgs::Ball ball_msg = *msg;

    Point new_position = Point(ball_msg.position.x, ball_msg.position.y);
    Point new_velocity = Point(ball_msg.velocity.x, ball_msg.velocity.y);

    Ball new_ball = Ball();
    new_ball.update(new_position, new_velocity);

    world.updateBall(new_ball);
}

void friendlyTeamUpdateCallback(const thunderbots_msgs::Team::ConstPtr &msg)
{
    thunderbots_msgs::Team friendly_team_msg = *msg;

    std::vector<Robot> friendly_robots = std::vector<Robot>();
    for (auto robot_msg : friendly_team_msg.robots)
    {
        Robot robot = Robot(robot_msg);
        friendly_robots.emplace_back(robot);
    }

    Team new_friendly_team = Team();
    new_friendly_team.update(friendly_robots);

    world.updateFriendlyTeam(new_friendly_team);
}

void enemyTeamUpdateCallback(const thunderbots_msgs::Team::ConstPtr &msg)
{
    thunderbots_msgs::Team enemy_team_msg = *msg;

    std::vector<Robot> enemy_robots = std::vector<Robot>();
    for (auto robot_msg : enemy_team_msg.robots)
    {
        Robot robot = Robot(robot_msg);
        enemy_robots.emplace_back(robot);
    }

    Team new_enemy_team = Team();
    new_enemy_team.update(enemy_robots);

    world.updateEnemyTeam(new_enemy_team);
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

    // Initialize variables
    world      = World();
    high_level = STPHL();

    // Main loop
    while (ros::ok())
    {
        std::vector<std::pair<unsigned int, Intent>> assignedIntents =
            high_level.getIntentAssignment(world);


        // Spin once to let all necessary callbacks run
        ros::spinOnce();
    }

    return 0;
}

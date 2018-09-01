#include <ros/ros.h>
#include "ai/ai.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/Primitive.h"
#include "thunderbots_msgs/PrimitiveArray.h"
#include "thunderbots_msgs/Team.h"
#include "util/constants.h"
#include "util/timestamp.h"

// Variables we need to maintain state
// In an anonymous namespace so they cannot be seen/accessed externally
namespace
{
    AI ai;
}


// Callbacks to update the state of the world
void fieldUpdateCallback(const thunderbots_msgs::Field::ConstPtr &msg)
{
    thunderbots_msgs::Field field_msg = *msg;

    ai.updateWorldFieldState(field_msg);
}

void ballUpdateCallback(const thunderbots_msgs::Ball::ConstPtr &msg)
{
    thunderbots_msgs::Ball ball_msg = *msg;

    ai.updateWorldBallState(ball_msg);
}

void friendlyTeamUpdateCallback(const thunderbots_msgs::Team::ConstPtr &msg)
{
    thunderbots_msgs::Team friendly_team_msg = *msg;

    ai.updateWorldFriendlyTeamState(friendly_team_msg);
}

void enemyTeamUpdateCallback(const thunderbots_msgs::Team::ConstPtr &msg)
{
    thunderbots_msgs::Team enemy_team_msg = *msg;

    ai.updateWorldEnemyTeamState(enemy_team_msg);
}

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "ai_logic");
    ros::NodeHandle node_handle;

    // Create publishers
    ros::Publisher primitive_publisher =
        node_handle.advertise<thunderbots_msgs::PrimitiveArray>(
            UTIL::CONSTANTS::AI_PRIMITIVES_TOPIC, 1);

    // Create subscribers
    ros::Subscriber field_sub = node_handle.subscribe(
        UTIL::CONSTANTS::BACKEND_INPUT_FIELD_TOPIC, 1, fieldUpdateCallback);
    ros::Subscriber ball_sub = node_handle.subscribe(
        UTIL::CONSTANTS::BACKEND_INPUT_BALL_TOPIC, 1, ballUpdateCallback);
    ros::Subscriber friendly_team_sub =
        node_handle.subscribe(UTIL::CONSTANTS::BACKEND_INPUT_FRIENDLY_TEAM_TOPIC, 1,
                              friendlyTeamUpdateCallback);
    ros::Subscriber enemy_team_sub = node_handle.subscribe(
        UTIL::CONSTANTS::BACKEND_INPUT_ENEMY_TEAM_TOPIC, 1, enemyTeamUpdateCallback);

    // Initialize variables used to maintain state
    ai = AI();

    // Main loop
    while (ros::ok())
    {
        // Spin once to let all necessary callbacks run
        // These callbacks will update the AI's world state
        ros::spinOnce();

        // Get the Primitives the Robots should run from the AI
        // We pass a timestamp with the current time (the time we initiate the call)
        // to let the AI update its predictors so that decisions are always made with the
        // most up to date predicted data (eg. future Robot or Ball position), even if
        // some time has passed since the AI's state was last updated.
        AITimestamp timestamp = Timestamp::getTimestampNow();
        std::vector<std::unique_ptr<Primitive>> assignedPrimitives =
            ai.getPrimitives(timestamp);

        // Put these Primitives into a message and publish it
        thunderbots_msgs::PrimitiveArray primitive_array_message;
        for (auto const &prim : assignedPrimitives)
        {
            thunderbots_msgs::Primitive msg = prim->createMsg();
            primitive_array_message.primitives.emplace_back(msg);
            std::cout << msg << std::endl;
        }
        primitive_publisher.publish(primitive_array_message);
    }

    return 0;
}

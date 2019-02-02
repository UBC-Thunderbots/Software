#include <ros/ros.h>
#include <thunderbots_msgs/RefboxData.h>

#include "ai/ai.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/Primitive.h"
#include "thunderbots_msgs/PrimitiveArray.h"
#include "thunderbots_msgs/Team.h"
#include "util/constants.h"
#include "util/logger/init.h"
#include "util/parameter/dynamic_parameters.h"
#include "util/ros_messages.h"
#include "util/timestamp.h"
#include "util/visualizer_messenger/visualizer_messenger.h"

// Member variables we need to maintain state
// They are kept in an anonymous namespace so they are not accessible outside this
// file and are not created as global static variables.
namespace
{
    // Initialize our AI, which is the main object that maintains state
    AI ai = AI(
        World(Field(0, 0, 0, 0, 0, 0, 0), Ball(Point(), Vector()),
              Team(std::chrono::milliseconds(
                  Util::DynamicParameters::robot_expiry_buffer_milliseconds.value())),
              Team(std::chrono::milliseconds(
                  Util::DynamicParameters::robot_expiry_buffer_milliseconds.value()))));
}  // namespace

// Callbacks to update the state of the world
void fieldUpdateCallback(const thunderbots_msgs::Field::ConstPtr &msg)
{
    thunderbots_msgs::Field field_msg = *msg;

    Field field = Util::ROSMessages::createFieldFromROSMessage(field_msg);

    ai.updateWorldFieldState(field);
}

void ballUpdateCallback(const thunderbots_msgs::Ball::ConstPtr &msg)
{
    thunderbots_msgs::Ball ball_msg = *msg;

    Ball ball = Util::ROSMessages::createBallFromROSMessage(ball_msg);

    ai.updateWorldBallState(ball);
}

void friendlyTeamUpdateCallback(const thunderbots_msgs::Team::ConstPtr &msg)
{
    thunderbots_msgs::Team friendly_team_msg = *msg;

    Team friendly_team = Util::ROSMessages::createTeamFromROSMessage(friendly_team_msg);

    ai.updateWorldFriendlyTeamState(friendly_team);
}

void enemyTeamUpdateCallback(const thunderbots_msgs::Team::ConstPtr &msg)
{
    thunderbots_msgs::Team enemy_team_msg = *msg;

    Team enemy_team = Util::ROSMessages::createTeamFromROSMessage(enemy_team_msg);

    ai.updateWorldEnemyTeamState(enemy_team);
}

void refboxGameStateUpdateCallback(const thunderbots_msgs::RefboxData::ConstPtr &msg)
{
    thunderbots_msgs::RefboxCommand command = msg->command;
    RefboxGameState game_state =
        Util::ROSMessages::createGameStateFromROSMessage(command);
    ai.updateWorldRefboxGameState(game_state);
}

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "ai_logic");
    ros::NodeHandle node_handle;

    // Create publishers
    ros::Publisher primitive_publisher =
        node_handle.advertise<thunderbots_msgs::PrimitiveArray>(
            Util::Constants::AI_PRIMITIVES_TOPIC, 1);

    // Create subscribers
    ros::Subscriber field_sub = node_handle.subscribe(
        Util::Constants::NETWORK_INPUT_FIELD_TOPIC, 1, fieldUpdateCallback);
    ros::Subscriber ball_sub = node_handle.subscribe(
        Util::Constants::NETWORK_INPUT_BALL_TOPIC, 1, ballUpdateCallback);
    ros::Subscriber friendly_team_sub =
        node_handle.subscribe(Util::Constants::NETWORK_INPUT_FRIENDLY_TEAM_TOPIC, 1,
                              friendlyTeamUpdateCallback);
    ros::Subscriber enemy_team_sub = node_handle.subscribe(
        Util::Constants::NETWORK_INPUT_ENEMY_TEAM_TOPIC, 1, enemyTeamUpdateCallback);
    ros::Subscriber game_state_sub =
        node_handle.subscribe(Util::Constants::NETWORK_INPUT_GAMECONTROLLER_TOPIC, 1,
                              refboxGameStateUpdateCallback);

    // Initialize the logger
    Util::Logger::LoggerSingleton::initializeLogger(node_handle);

    // Initialize the draw visualizer messenger
    Util::VisualizerMessenger::getInstance()->initializePublisher(node_handle);

    // Main loop
    while (ros::ok())
    {
        // Spin once to let all necessary callbacks run
        // These callbacks will update the AI's world state
        ros::spinOnce();
        try
        {
            // Get the Primitives the Robots should run from the AI
            // We pass a timestamp with the current time (the time we initiate the call)
            // to let the AI update its predictors so that decisions are always made with
            // the most up to date predicted data (eg. future Robot or Ball position),
            // even if some time has passed since the AI's state was last updated.
            AITimestamp timestamp = Timestamp::getTimestampNow();
            std::vector<std::unique_ptr<Primitive>> assignedPrimitives =
                ai.getPrimitives(timestamp);

            // Put these Primitives into a message and publish it
            thunderbots_msgs::PrimitiveArray primitive_array_message;
            for (auto const &prim : assignedPrimitives)
            {
                thunderbots_msgs::Primitive msg = prim->createMsg();
                primitive_array_message.primitives.emplace_back(msg);
                LOG(INFO) << msg << std::endl;
            }
            primitive_publisher.publish(primitive_array_message);

            // CHANGED: on every tick, push testing layer objects
            Util::VisualizerMessenger::testShapes();

            // On every tick, send the layer messages
            Util::VisualizerMessenger::getInstance()->publishAndClearLayers();
        }
        catch (const std::invalid_argument &e)
        {
            std::cout << e.what() << std::endl;
        }
    }

    return 0;
}

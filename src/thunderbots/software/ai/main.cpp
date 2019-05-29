#include <ros/ros.h>

#include "ai/ai.h"
#include "thunderbots_msgs/PrimitiveArray.h"
#include "thunderbots_msgs/World.h"
#include "util/canvas_messenger/canvas_messenger.h"
#include "util/constants.h"
#include "util/logger/init.h"
#include "util/parameter/dynamic_parameter_utils.h"
#include "util/parameter/dynamic_parameters.h"
#include "util/ros_messages.h"
#include "util/time/timestamp.h"

// Member variables we need to maintain state
// They are kept in an anonymous namespace so they are not accessible outside this
// file and are not created as global static variables.
namespace
{
    // The publisher used to send new Primitive commands
    ros::Publisher primitive_publisher;
    // Our instance of the AI that decides what Primitives to run
    AI ai;
}  // namespace

int count;

// Runs the AI and sends new Primitive commands every time we get new information
// about the World
void worldUpdateCallback(const thunderbots_msgs::World::ConstPtr &msg)
{
    if (!Util::DynamicParameters::AI::run_ai.value())
    {
        return;
    }

    thunderbots_msgs::World world_msg = *msg;
    World world = Util::ROSMessages::createWorldFromROSMessage(world_msg);

    // Get the Primitives the Robots should run from the AI
    std::vector<std::unique_ptr<Primitive>> assignedPrimitives = ai.getPrimitives(world);

    // Put these Primitives into a message and publish it
    thunderbots_msgs::PrimitiveArray primitive_array_message;
    for (auto const &prim : assignedPrimitives)
    {
        primitive_array_message.primitives.emplace_back(prim->createMsg());
    }
    primitive_publisher.publish(primitive_array_message);

    // Draw the world
    std::shared_ptr<Util::CanvasMessenger> canvas_messenger =
        Util::CanvasMessenger::getInstance();
    canvas_messenger->drawWorld(world);

    count++;
}

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "ai_logic");
    ros::NodeHandle node_handle;

    // Create publishers
    primitive_publisher = node_handle.advertise<thunderbots_msgs::PrimitiveArray>(
        Util::Constants::AI_PRIMITIVES_TOPIC, 1);

    // Create subscribers
    ros::Subscriber world_subscriber = node_handle.subscribe(
        Util::Constants::NETWORK_INPUT_WORLD_TOPIC, 1, worldUpdateCallback);

    // Initialize the logger
    Util::Logger::LoggerSingleton::initializeLogger(node_handle);

    // Initialize the draw visualizer messenger
    Util::CanvasMessenger::getInstance()->initializePublisher(node_handle);

    // Initialize Dynamic Parameters
    auto update_subscribers =
        Util::DynamicParameters::initUpdateSubscriptions(node_handle);

    // Services any ROS calls in a separate thread "behind the scenes". Does not return
    // until the node is shutdown
    // http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::spin();

    return 0;
}

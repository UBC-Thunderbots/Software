#include <ros/ros.h>
#include <thunderbots_msgs/Pass.h>
#include <thunderbots_msgs/Circle2D.h>
#include <thunderbots_msgs/World.h>
#include <util/ros_messages.h>
#include <ai/world/world.h>
#include "util/constants.h"
#include "util/logger/init.h"
#include "util/parameter/dynamic_parameters.h"
#include "util/visualizer_messenger/visualizer_messenger.h"
#include "geom/circle.h"


// Member variables we need to maintain state
// They are kept in an anonymous namespace so they are not accessible outside this
// file and are not created as global static variables.
namespace {
    World world;
    Circle desired_pass_region;
}

// Callback to update the state of the world
void worldUpdateCallback(const thunderbots_msgs::World::ConstPtr &msg)
{
    thunderbots_msgs::World world_msg = *msg;

    World world = Util::ROSMessages::createWorldFromROSMessage(world_msg);
}

// Callback to update the desired pass region
void desiredRegionUpdateCallback(const thunderbots_msgs::Circle2D::ConstPtr &msg)
{
    thunderbots_msgs::Circle2D circle_msg = *msg;

    Circle circle = Util::ROSMessages::createCircleFromROSMessage(circle_msg);
}

int main(int argc, char** argv) {
    // Init ROS node
    ros::init(argc, argv, "passing_gradient_descent");
    ros::NodeHandle node_handle;

    // Create Publishers
    ros::Publisher pass_publisher = node_handle.advertise<thunderbots_msgs::Pass>(
            Util::Constants::PASSING_GRADIENT_DESCENT_PASS_TOPIC, 1
            );

    // Create Subscribers
    ros::Subscriber desired_pass_region_subscriber =
            node_handle.subscribe<thunderbots_msgs::Circle2D>(
            Util::Constants::AI_DESIRED_PASS_REGION, 1, desiredRegionUpdateCallback);

    // Initialize the logger
    Util::Logger::LoggerSingleton::initializeLogger(node_handle);

    // Initialize the draw visualizer messenger
    Util::VisualizerMessenger::getInstance()->initializePublisher(node_handle);

    // Main loop
    while (ros::ok()) {
        // TODO: do gradient descent here
    }
}
#include <ros/ros.h>
#include <thunderbots_msgs/Primitive.h>
#include <thunderbots_msgs/PrimitiveArray.h>
#include <thunderbots_msgs/World.h>
#include "ai/primitive/primitive.h"
#include "ai/primitive/primitive_factory.h"
#include "mrf_backend.h"
#include "util/constants.h"
#include "util/logger/init.h"
#include "util/ros_messages.h"

// Member variables we need to maintain state
// They are kept in an anonymous namespace so they are not accessible outside this
// file and are not created as global static variables.
namespace
{
    // The MRFBackend instance that connects to the dongle
    MRFBackend backend = MRFBackend();
}  // namespace

// Callbacks
void primitiveUpdateCallback(const thunderbots_msgs::PrimitiveArray::ConstPtr& msg)
{
    std::vector<std::unique_ptr<Primitive>> primitives;
    thunderbots_msgs::PrimitiveArray prim_array_msg = *msg;
    for (const thunderbots_msgs::Primitive& prim_msg : prim_array_msg.primitives)
    {
        primitives.emplace_back(AI::Primitive::createPrimitiveFromROSMessage(prim_msg));
    }

    // Send primitives
    backend.sendPrimitives(primitives);
}

void worldUpdateCallback(const thunderbots_msgs::World::ConstPtr& msg)
{
    thunderbots_msgs::World world_msg = *msg;

    // Extract team and ball data
    Team friendly_team =
        Util::ROSMessages::createTeamFromROSMessage(world_msg.friendly_team);
    Ball ball = Util::ROSMessages::createBallFromROSMessage(world_msg.ball);

    std::vector<std::tuple<uint8_t, Point, Angle>> robots;
    for (const Robot& r : friendly_team.getAllRobots())
    {
        robots.emplace_back(std::make_tuple(r.id(), r.position(), r.orientation()));
    }

    // Update robots and ball
    backend.update_robots(robots);
    backend.update_ball(ball);

    // Send vision packet
    backend.send_vision_packet();

    // Handle libusb events for the dongle
    backend.update_dongle_events();
}

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "radio_communication");
    ros::NodeHandle node_handle;

    // Create subscribers to topics we care about
    ros::Subscriber primitive_subscriber = node_handle.subscribe(
        Util::Constants::AI_PRIMITIVES_TOPIC, 1, primitiveUpdateCallback);
    ros::Subscriber world_sub = node_handle.subscribe(
        Util::Constants::NETWORK_INPUT_WORLD_TOPIC, 1, worldUpdateCallback);

    // Initialize the logger
    Util::Logger::LoggerSingleton::initializeLogger(node_handle);

    ros::spin();

    // Services any ROS calls in a separate thread "behind the scenes". Does not return
    // until the node is shutdown
    // http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    return 0;
}

#include <ros/ros.h>
#include <ros/time.h>
#include <thunderbots_msgs/Primitive.h>
#include <thunderbots_msgs/PrimitiveArray.h>
#include <thunderbots_msgs/World.h>

#include "ai/primitive/primitive.h"
#include "ai/primitive/primitive_factory.h"
#include "geom/point.h"
#include "mrf_backend.h"
#include "util/constants.h"
#include "util/logger/init.h"
#include "util/parameter/dynamic_parameter_utils.h"
#include "util/parameter/dynamic_parameters.h"
#include "util/ros_messages.h"


namespace
{
    // A vector of primitives. It is cleared each tick, populated by the callbacks
    // that receive primitive commands, and is processed by the backend to send primitives
    // to the robots over radio.
    std::vector<std::unique_ptr<Primitive>> primitives;

    // The MRFBackend instance that connects to the dongle
    MRFBackend backend = MRFBackend();
}  // namespace

// Callbacks
void primitiveUpdateCallback(const thunderbots_msgs::PrimitiveArray::ConstPtr& msg)
{
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
        robots.push_back(std::make_tuple(r.id(), r.position(), r.orientation()));
    }

    // Update robots and ball
    backend.update_robots(robots);
    backend.update_ball(ball);

    // Send vision packet
    backend.send_vision_packet();
}

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "radio_communication");
    ros::NodeHandle node_handle;

    // Create subscribers to topics we care about
    ros::Subscriber prim_array_sub = node_handle.subscribe(
        Util::Constants::AI_PRIMITIVES_TOPIC, 1, primitiveUpdateCallback);
    ros::Subscriber world_sub = node_handle.subscribe(
        Util::Constants::NETWORK_INPUT_WORLD_TOPIC, 1, worldUpdateCallback);

    // Initialize the logger
    Util::Logger::LoggerSingleton::initializeLogger(node_handle);

    // Initialize variables
    primitives = std::vector<std::unique_ptr<Primitive>>();

    // Initialize Dynamic Parameters
    auto update_subscribers =
        Util::DynamicParameters::initUpdateSubscriptions(node_handle);

    // Main loop
    while (ros::ok())
    {
        // Clear all primitives each tick
        primitives.clear();

        // Handle libusb events for the dongle
        backend.update_dongle_events();

        // Spin once to let all necessary callbacks run
        // The callbacks will populate the primitives vector
        ros::spinOnce();
    }

    return 0;
}

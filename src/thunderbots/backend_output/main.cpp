#include <ros/ros.h>
#include <ros/time.h>
#include <thunderbots_msgs/MovePrimitive.h>
#include <iostream>
#include "ai/primitive/move_prim.h"
#include "ai/primitive/primitive.h"
#include "backend_output/grsim/grsim_backend.h"
#include "geom/point.h"

// Member variables we need to maintain state

// A vector of primitives. It is cleared each tick, populated by the callbacks
// that receive primitive commands, and is processed by the backend to send
// the primitives to the system we have chosen (such as grSim, our radio, etc.)
std::vector<Primitive> primitives;

// Callbacks
void movePrimUpdateCallback(const thunderbots_msgs::MovePrimitive::ConstPtr& msg)
{
    thunderbots_msgs::MovePrimitive move_prim_msg = *msg;
    MovePrim move_prim                            = MovePrim(move_prim_msg);
    primitives.push_back(move_prim);
}

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "backend_output");
    ros::NodeHandle node_handle;

    // Create subscribers to topics we care about
    ros::Subscriber move_prim_sub =
        node_handle.subscribe("backend/move_prim", 1, movePrimUpdateCallback);

    // Initialize variables
    primitives           = std::vector<Primitive>();
    GrSimBackend backend = GrSimBackend("127.0.0.1", 20011);

    // We loop at 30Hz so we don't overload the network with too many packets
    ros::Rate tick_rate(30);

    // Main loop
    while (ros::ok())
    {
        // Clear all primitives each tick
        primitives.clear();

        // Spin once to let all necessary callbacks run
        // The callbacks will populate the primitives vector
        ros::spinOnce();

        backend.sendPrimitives(primitives);

        tick_rate.sleep();
    }

    return 0;
}
